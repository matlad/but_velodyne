/**
 * @file     Calibrator.cpp
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */


#include "but_calibration_camera_velodyne/Calibrator.h"
#include "but_calibration_camera_velodyne/Calibration6DoF.h"

#include <utility>
#include <vector>
#include <rosconsole/macros_generated.h>
#include <ros/console_backend.h>
#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>

#define INPUT_STORACE "/media/Linux_Data/Code/school/FIT_BAK_COLOR_LINDAR/src/but_velodyne/but_calibration_camera_velodyne/data/"

using namespace But::calibration_camera_velodyne;
using cv::Mat;
using std::vector;
using ::But::calibration_camera_velodyne::Velodyne::Velodyne;
using ::But::calibration_camera_velodyne::Image::Image;
using Velodyne::Processing;
using cv::Point2f;
using cv::Point3f;


/**
 * @brief Nastaví obraz kamery pro kalibraci
 * @param image
 */
void Calibrator::setImage(cv::Mat image) {
  this->image = std::move(image);
  undisortImage();
  cv::rotate(this->image, this->image, cv::ROTATE_90_CLOCKWISE);

  Mat frame_gray;
  cvtColor(this->image, frame_gray, CV_BGR2GRAY);
  auto img = Image::Image(frame_gray);
  this->edgeImage = new Image::Image(img.computeEdgeImage());
  //SHOW_IMAGE(this->image,"Input image")
}

/**
 * @brief Nastaví point cloud pro klaibraci
 * @param pointCloud
 */
void Calibrator::setPointCloud(pcl::PointCloud<But::calibration_camera_velodyne::Velodyne::Point> pointCloud) {
  this->pointCloud = Velodyne::Velodyne(pointCloud);
  this->pointCloud = this->pointCloud.transform(0, 0, 0, M_PI / 2, 0, 0);
  this->edgePointCloud = Velodyne::Velodyne(this->pointCloud);
  edgePointCloud.intensityByDiff(Processing::DISTORTIONS);
  edgePointCloud.view(0);
//  this->pointCloud.normalizeIntensity(0.,1.);
//  this->pointCloud.view(0);
}

/**
 * @brief Nastaví (vnitřní parametry kamery)
 * @param cameraParameters
 */
void Calibrator::setCameraParameters(CameraParameters &cameraParameters) {
  this->cameraParameters = cameraParameters;
}

/**
 * @brief provede opravu zkreslení obrazu
 */
void Calibrator::undisortImage() {
  cv::fisheye::undistortImage(
	  image,
	  image,
	  cameraParameters.K,
	  cameraParameters.D,
	  cameraParameters.K
  );
}

Calibration6DoF Calibrator::calibration(bool doRefinement = false) {

  Mat frame_gray;
  cvtColor(image, frame_gray, CV_BGR2GRAY);

  auto P = cameraParameters.P;

  But::calibration_camera_velodyne::Calibration3DMarker calibration3DMarker(
	  frame_gray,
	  P,
	  pointCloud.getPointCloud(),
	  circleDistance,
	  radius
  );

  vector<float> radii2D;
  vector<Point2f> centers2D;

  vector<float> radii3D;
  vector<Point3f> centers3D;
  bool detectedInImage =
	  calibration3DMarker.detectCirclesInImage(centers2D, radii2D);
  bool detectedInPointCloud =
	  calibration3DMarker.detectCirclesInPointCloud(centers3D, radii3D);

  pointCloud.viewMarker(centers3D,radii3D,"");

  if (!detectedInImage) {
	ROS_INFO_STREAM("Detection in image failed");
  }

  if (!detectedInPointCloud) {
	ROS_INFO_STREAM("Detection in poitcloud failed");
  }

  if (!detectedInPointCloud || !detectedInImage) {
	return Calibration6DoF::wrong();
  }

  Mat mrvec(1, 3, CV_64FC1);
  Mat mtvec(1, 3, CV_64FC1);

  cout << "jdeme na pnp" << endl;

  cv::Mat c3d(3, centers3D.size(), CV_32FC1);

  for (size_t i = 0, end = centers3D.size(); i < end; ++i) {
	c3d.at<float>(0, i) = centers3D[i].x;
	c3d.at<float>(1, i) = centers3D[i].y;
	c3d.at<float>(2, i) = centers3D[i].z;
  }

  cv::Mat c2d(2, centers2D.size(), CV_32FC1);

  for (size_t i = 0, end = centers2D.size(); i < end; ++i) {
	c2d.at<float>(0, i) = centers2D[i].x;
	c2d.at<float>(1, i) = centers2D[i].y;
  }

  Calibration6DoF calib;
  solvePnP(centers3D, centers2D, cameraParameters.K, Mat(), mrvec, mtvec);

  calib.set(mtvec, mrvec, getSimilarity(mtvec, mtvec));
  calib.print();

  //centers3D.pop_back();
  //centers2D.pop_back();

  solvePnP(centers3D,
		   centers2D,
		   cameraParameters.K,
		   Mat(),
		   mrvec,
		   mtvec,
		   false,
		   cv::SOLVEPNP_P3P);

  calib.set(mtvec, mrvec, getSimilarity(mtvec, mtvec));
  calib.print();

  solvePnP(centers3D,
		   centers2D,
		   cameraParameters.K,
		   Mat(),
		   mrvec,
		   mtvec,
		   false,
		   cv::SOLVEPNP_AP3P);

  calib.set(mtvec, mrvec, getSimilarity(mtvec, mtvec));
  calib.print();

  float radius2D =
	  accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  float radius3D =
	  accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

  // rough calibration
  auto translation = findTranslation(centers2D, centers3D, radius2D, radius3D);

  solvePnPRansac(centers3D,
				 centers2D,
				 cameraParameters.K,
				 Mat(),
				 mrvec,
				 mtvec,
				 false,
				 100,
				 8.0);

  calib.set(mtvec, mrvec, getSimilarity(mtvec, mtvec));
  calib.print();

  Mat tvec(1, 3, CV_32FC1, translation.DoF.data());
  Mat rvec(1, 3, CV_32FC1, translation.DoF.data() + 3);

  solvePnPRansac(centers3D,
				 centers2D,
				 cameraParameters.K,
				 Mat(),
				 rvec,
				 tvec,
				 true,
				 100,
				 8.0);

  calib.set(tvec, rvec, getSimilarity(tvec, tvec));
  calib.print();

  if (doRefinement) {
	ROS_INFO("Coarse calibration:");
	translation.print();
	ROS_INFO("Refinement process started - this may take a minute.");
	size_t divisions = 5;
	float distance_transl = 0.02;
	float distance_rot = 0.01;
	Calibration6DoF best_calibration, avg_calibration;
	calibrationRefinement(
		translation,
		distance_transl,
		distance_rot,
		divisions,
		best_calibration,
		avg_calibration
	);
	best_calibration.print();
	return avg_calibration;
  } else {
	return translation;
  }
}
Calibrator::Calibrator(float circleDistance, float radius) : circleDistance(
	circleDistance), radius(radius) {}

float Calibrator::getSimilarity(cv::Mat &tvec, cv::Mat &rvec) {
  Mat frame_gray;
  auto transformed = edgePointCloud.transform(tvec, rvec);
  return Similarity::edgeSimilarity(
	  *edgeImage,
	  transformed,
	  cameraParameters.P
  );
}

Calibrator::~Calibrator() {
  delete edgeImage;
}



Calibration6DoF Calibrator::findTranslation(std::vector<cv::Point2f> image,
											std::vector<cv::Point3f> velodyne,
											float radius2D,
											float radius3D) {
  auto projection = cameraParameters.P;

  Mat rvec(3, 1, CV_64FC1);
  rvec.at<float>(0,0) = 0;
  rvec.at<float>(0,1) = 0;
  rvec.at<float>(0,2) = 0;

  Mat tvec(3, 1, CV_64FC1);

	auto X = cv::Point(0,0);
	auto Y = cv::Point(0,1);
  	auto Z = cv::Point(0,2);
  float focal_len = projection.at<float>(0, 0);

  // t_z:
  tvec.at<float>(Z) = radius3D * focal_len / radius2D - velodyne.front().z;

  float principal_x = projection.at<float>(0, 2);
  float principal_y = projection.at<float>(1, 2);

  for (size_t i = 0; i < image.size(); i++)
  {
	// t_x:
	tvec.at<float>(X) += (image[i].x - principal_x) * (velodyne[i].z + tvec.at<float>(Z)) / focal_len
		- velodyne[i].x;
	// t_y:
	tvec.at<float>(Y) += (image[i].y - principal_y) * (velodyne[i].z + tvec.at<float>(Z)) / focal_len
		- velodyne[i].y;
  }
  tvec.at<float>(X) /= image.size();
  tvec.at<float>(Y) /= image.size();



  // no rotation and value of calibration
  return Calibration6DoF(tvec, rvec, getSimilarity(tvec,rvec));
}

void Calibrator::calibrationRefinement(
	Calibration6DoF &rough,
	float max_translation,
	float max_rotation,
	unsigned steps,
	Calibration6DoF &best_calibration,
	Calibration6DoF &average
) {

  auto x_rough = rough.tX();
  auto y_rough = rough.tY();
  auto z_rough = rough.tZ();

  auto P = cameraParameters.P;

  auto image = Image::Image(this->image);
  image = Image::Image(image.computeIDTEdgeImage());

  float x_min = x_rough - max_translation;
  float y_min = y_rough - max_translation;
  float z_min = z_rough - max_translation;
  float x_rot_min = -max_rotation;
  float y_rot_min = -max_rotation;
  float z_rot_min = -max_rotation;

  float step_transl = max_translation * 2 / (steps - 1);
  float step_rot = max_rotation * 2 / (steps - 1);

  Velodyne::Velodyne transformed = edgePointCloud.transform(x_rough, y_rough, z_rough, 0, 0, 0);
  float rough_val = Similarity::edgeSimilarity(image, transformed, P);

  best_calibration.set(x_rough, y_rough, z_rough, 0, 0, 0, rough_val);

  int counter = 0;

  float x = x_min;
  for (size_t xi = 0; xi < steps; xi++)
  {

	float y = y_min;
	for (size_t yi = 0; yi < steps; yi++)
	{

	  float z = z_min;
	  for (size_t zi = 0; zi < steps; zi++)
	  {

		float x_r = x_rot_min;
		for (size_t x_ri = 0; x_ri < steps; x_ri++)
		{

		  float y_r = y_rot_min;
		  for (size_t y_ri = 0; y_ri < steps; y_ri++)
		  {

			float z_r = z_rot_min;
			for (size_t z_ri = 0; z_ri < steps; z_ri++)
			{

			  transformed = pointCloud.transform(x, y, z, x_r, y_r, z_r);
			  float value = Similarity::edgeSimilarity(image, transformed, P);
			  Calibration6DoF calibration(x, y, z, x_r, y_r, z_r, value);
			  if (value > best_calibration.value)
			  {
				best_calibration.set(x, y, z, x_r, y_r, z_r, value);
			  }

			  if (value > rough_val)
			  {
				average += calibration;
				counter++;
			  }
			  //cout << counter << ".\t";
			  //calibration.print();

			  z_r += step_rot;
			}
			y_r += step_rot;
		  }
		  x_r += step_rot;
		}
		z += step_transl;
	  }
	  y += step_transl;
	}
	x += step_transl;
  }
  average /= counter;
}

