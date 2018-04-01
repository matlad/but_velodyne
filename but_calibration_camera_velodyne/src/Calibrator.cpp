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

using namespace But::calibration_camera_velodyne;
using cv::Mat;
using std::vector;
using but_calibration_camera_velodyne::Velodyne::Velodyne;
using but_calibration_camera_velodyne::Calibration;
using but_calibration_camera_velodyne::Image::Image;
using but_calibration_camera_velodyne::Similarity;
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
  auto img = Image(frame_gray);
  this->edgeImage = new Image(img.computeIDTEdgeImage());
  //SHOW_IMAGE(this->image,"Input image")
}

/**
 * @brief Nastaví point cloud pro klaibraci
 * @param pointCloud
 */
void Calibrator::setPointCloud(pcl::PointCloud<but_calibration_camera_velodyne::Velodyne::Point> pointCloud) {
  this->pointCloud = Velodyne(pointCloud);
  this->pointCloud = this->pointCloud.transform(0, 0, 0, M_PI / 2, 0, 0);
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

  but_calibration_camera_velodyne::Calibration3DMarker calibration3DMarker(
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

  auto B = Mat(1, 4, CV_32FC1, {0, 0, 0, 0});


  solvePnP(centers3D, centers2D, cameraParameters.K, Mat(), mrvec, mtvec);

  auto calib = Calibration6DoF(
	  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
	  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2),
	  getSimilarity(
		  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
		  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2)
	  )
  );

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

  calib = Calibration6DoF(
	  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
	  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2),
	  getSimilarity(
		  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
		  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2)
	  )
  );
  calib.print();

  solvePnP(centers3D,
		   centers2D,
		   cameraParameters.K,
		   Mat(),
		   mrvec,
		   mtvec,
		   false,
		   cv::SOLVEPNP_AP3P);

  calib = Calibration6DoF(
	  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
	  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2),
	  getSimilarity(
		  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
		  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2)
	  )
  );
  calib.print();

  float radius2D =
	  accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  float radius3D =
	  accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

  // rough calibration
  Calibration6DoF translation = Calibration::findTranslation(centers2D,
															 centers3D,
															 cameraParameters.P,
															 radius2D,
															 radius3D);

  solvePnPRansac(centers3D,
				 centers2D,
				 cameraParameters.K,
				 Mat(),
				 mrvec,
				 mtvec,
				 false,
				 100,
				 8.0);
  calib = Calibration6DoF(
	  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
	  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2),
	  getSimilarity(
		  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
		  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2)
	  )
  );
  calib.print();

  Mat tvec(1, 3, CV_32FC1, translation.DoF.data());
  Mat rvec(1, 3, CV_32FC1, translation.DoF.data() + 3);

  solvePnPRansac(centers3D,
				 centers2D,
				 cameraParameters.K,
				 Mat(),
				 mrvec,
				 mtvec,
				 true,
				 100,
				 8.0);
  calib = Calibration6DoF(
	  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
	  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2),
	  getSimilarity(
		  mtvec.at<float>(0, 0), mtvec.at<float>(0, 1), mtvec.at<float>(0, 2),
		  mrvec.at<float>(0, 0), mrvec.at<float>(0, 1), mrvec.at<float>(0, 2)
	  )
  );
  calib.print();

  if (doRefinement) {
	ROS_INFO("Coarse calibration:");
	translation.print();
	ROS_INFO("Refinement process started - this may take a minute.");
	size_t divisions = 5;
	float distance_transl = 0.02;
	float distance_rot = 0.01;
	Calibration6DoF best_calibration, avg_calibration;
	Calibration::calibrationRefinement(Image(frame_gray),
									   pointCloud,
									   cameraParameters.P,
									   translation.DoF[0],
									   translation.DoF[1],
									   translation.DoF[2],
									   distance_transl,
									   distance_rot,
									   divisions,
									   best_calibration,
									   avg_calibration);
	return avg_calibration;
  } else {
	return translation;
  }
}
Calibrator::Calibrator(float circleDistance, float radius) : circleDistance(
	circleDistance), radius(radius) {}


float Calibrator::getSimilarity(
	float &x,
	float &y,
	float &z,
	float &x_r,
	float &y_r,
	float &z_r
) {
  Mat frame_gray;
  auto transformed = pointCloud.transform(x, y, z, x_r, y_r, z_r);
  return Similarity::edgeSimilarity(*edgeImage, transformed, cameraParameters.P);
}

Calibrator::~Calibrator() {
	delete edgeImage;
}



