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
#include <but_calibration_camera_velodyne/Constants.h>
#include <but_calibration_camera_velodyne/Colorizer.h>
#include <but_calibration_camera_velodyne/types.h>
#include <but_calibration_camera_velodyne/macros.h>
#include <opencv2/core/types.hpp>


#define INPUT_STORACE "/media/Linux_Data/Code/school/FIT_BAK_COLOR_LINDAR/src/but_velodyne/but_calibration_camera_velodyne/data/"

using namespace but::calibration_camera_velodyne;
using cv::Mat;
using std::vector;
using ::but::calibration_camera_velodyne::Velodyne::Velodyne;
using ::but::calibration_camera_velodyne::Image::Image;
using Velodyne::Processing;
using cv::Point2f;
using cv::Point3f;

/**
 * @brief Nastaví obraz kamery pro kalibraci
 * @param image
 */
void Calibrator::setImage(Mat image) {
  image.copyTo(rawImg);
  this->image = std::move(image);
  undistortImage();
  cv::rotate(this->image, this->image, cv::ROTATE_90_CLOCKWISE);
  // SHOW_IMAGE(this->image,"Input image")
  Mat frame_gray;
  cvtColor(this->image, frame_gray, CV_BGR2GRAY);
  auto img = Image::Image(frame_gray);
  this->edgeImage = new Image::Image(img.computeIDTEdgeImage());
}

/**
 * @brief Nastaví point cloud pro klaibraci
 * @param pointCloud
 */
void Calibrator::setPointCloud(pcl::PointCloud<but::calibration_camera_velodyne::Velodyne::Point> pointCloud) {
  this->rawPCl = Velodyne::Velodyne(pointCloud);
  this->pointCloud = Velodyne::Velodyne(pointCloud);
  this->pointCloud = this->pointCloud.transform(0, 0, 0, M_PI / 2, 0, 0);
  this->edgePointCloud = Velodyne::Velodyne(this->pointCloud);
  edgePointCloud.intensityByDiff(Processing::DISTORTIONS);
  //edgePointCloud.view(POINTCLOUD_EDGE_TRASH_HOLD,"Set Pointcloud");
//  this->pointCloud.normalizeIntensity(0.,1.);
  //this->pointCloud.view(0);
}

/**
 * @brief Nastaví (vnitřní parametry kamery)
 * @param camera
 */
void Calibrator::setCamera(Camera *camera) {
  this->camera = camera;
}

/**
 * @brief provede opravu zkreslení obrazu
 */
void Calibrator::undistortImage() {

  cv::fisheye::undistortImage(image, image, camera->K, camera->D, camera->K);
}

Calibration6DoF Calibrator::calibration(bool doRefinement = false) {

  Mat frame_gray;
  cvtColor(image, frame_gray, CV_BGR2GRAY);

  but::calibration_camera_velodyne::Calibration3DMarker calibration3DMarker(
      frame_gray,
      camera->P,
      pointCloud.getPointCloud(),
      circleDistance,
      radius
  );

  Colorizer colorizer;

  colorizer.setPointCloud(rawPCl);
  colorizer.setImage(rawImg);
  colorizer.setCamera(camera);

  vector<double> radii2D;
  vector<Point2f> centers2D;

  vector<double> radii3D;
  vector<Point3f> centers3D;
  bool detectedInImage =
      calibration3DMarker.detectCirclesInImage(centers2D, radii2D);
  bool detectedInPointCloud =
      calibration3DMarker.detectCirclesInPointCloud(centers3D, radii3D);

  //edgePointCloud.viewMarker(centers3D, radii3D, "Detected Circles in 3D");

  if (!detectedInImage) {
    ROS_INFO_STREAM("Detection in image failed");
  }

  if (!detectedInPointCloud) {
    ROS_INFO_STREAM("Detection in poitcloud failed");
  }

  if (!detectedInPointCloud || !detectedInImage) {
    return Calibration6DoF::wrong();
  }

  Mat frvec = VEC_3D;
  Mat ftvec = VEC_3D;

  cout << "jdeme na pnp" << endl;

  cv::Mat c3d(3, static_cast<int>(centers3D.size()), CV_64FC1);

  for (int i = 0, end = static_cast<int>(centers3D.size()); i < end; ++i) {
    c3d.at<double>(0, i) = centers3D[i].x;
    c3d.at<double>(1, i) = centers3D[i].y;
    c3d.at<double>(2, i) = centers3D[i].z;
  }

  cv::Mat c2d(2, static_cast<int>(centers2D.size()), CV_64FC1);

  for (int i = 0, end = static_cast<int>(centers2D.size()); i < end; ++i) {
    c2d.at<double>(0, i) = centers2D[i].x;
    c2d.at<double>(1, i) = centers2D[i].y;
  }

  Calibration6DoF calib;
  solvePnP(centers3D, centers2D, camera->K, Mat(), frvec, ftvec);

  calib.set(ftvec, frvec, getSimilarity2(ftvec, frvec, "solvePnP"));
  cout << "solvePnP" << endl;
  calib.print();

  solvePnP(
      centers3D,
      centers2D,
      camera->K,
      Mat(),
      frvec,
      ftvec,
      false,
      cv::SOLVEPNP_P3P
  );

  calib.set(ftvec, frvec, getSimilarity2(ftvec, frvec, "solvePnP - P3P"));
  cout << "solvePnP - P3P" << endl;
  calib.print();
  camera->rvec = frvec;
  camera->tvec = ftvec;
  auto colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  solvePnP(centers3D,
           centers2D,
           camera->K,
           Mat(),
           frvec,
           ftvec,
           false,
           cv::SOLVEPNP_AP3P);

  calib.set(ftvec, frvec, getSimilarity2(ftvec, frvec, "solvePnP - AP3P"));
  cout << "solvePnP - AP3P" << endl;
  calib.print();
  camera->rvec = frvec;
  camera->tvec = ftvec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  double radius2D =
      accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  double radius3D =
      accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

  // rough calibration
  auto translation = findTranslation(centers2D, centers3D, radius2D, radius3D);
  cout << "findTranslation" << endl;
  translation.print();
  camera->rvec = frvec;
  camera->tvec = ftvec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  frvec.at<double>(X) = translation.rX();
  frvec.at<double>(Y) = translation.rY();
  frvec.at<double>(Z) = translation.rZ();
  ftvec.at<double>(X) = translation.tX();
  ftvec.at<double>(Y) = translation.tY();
  ftvec.at<double>(Z) = translation.tZ();

  camera->rvec = frvec;
  camera->tvec = ftvec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  solvePnPRansac(centers3D,
                 centers2D,
                 camera->K,
                 Mat(),
                 frvec,
                 ftvec,
                 false,
                 100,
                 8.0);

  calib.set(ftvec, frvec, getSimilarity2(ftvec, frvec, "solvePnPRansac"));
  cout << "solvePnPRansac" << endl;
  calib.print();

  Mat tvec(1, 3, CV_64FC1, translation.DoF.data());
  Mat rvec(1, 3, CV_64FC1, translation.DoF.data() + 3);

  solvePnPRansac(centers3D,
                 centers2D,
                 camera->K,
                 Mat(),
                 rvec,
                 tvec,
                 true,
                 100,
                 8.0);

  calib.set(tvec,
            rvec,
            getSimilarity2(tvec, rvec, "solvePnPRansac + findTranslation"));
  cout << "solvePnPRansac + findTranslation" << endl;
  calib.print();
  camera->rvec = frvec;
  camera->tvec = ftvec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  centers3D.pop_back();
  centers2D.pop_back();

  std::vector<Mat> rvecs, tvecs;

  cv::solveP3P(centers3D,
               centers2D,
               camera->K,
               Mat(),
               rvecs,
               tvecs,
               cv::SOLVEPNP_P3P);

  for (unsigned int i = 0; i < rvecs.size(); ++i) {
    calib.set(tvecs[i], rvecs[i], getSimilarity2(ftvec, frvec, "solveP3P"));
    cout << "solveP3P " << i << endl;
    calib.print();
  }

  cv::solveP3P(centers3D,
               centers2D,
               camera->K,
               Mat(),
               rvecs,
               tvecs,
               cv::SOLVEPNP_AP3P);

  for (unsigned int i = 0; i < rvecs.size(); ++i) {
    calib.set(tvecs[i], rvecs[i], getSimilarity2(ftvec, frvec, "solveAP3P"));
    cout << "solveAP3P " << i << endl;
    calib.print();
  }

  if (doRefinement) {
    ROS_INFO("Coarse calibration:");
    translation.print();
    ROS_INFO("Refinement process started - this may take a minute.");
    u_int divisions = 5;
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

Calibrator::Calibrator(double circleDistance, double radius) : circleDistance(
    circleDistance), radius(radius) {}

double Calibrator::getSimilarity(cv::Mat &tvec,
                                 cv::Mat &rvec,
                                 const char *title) {

  assert(tvec.cols == 1);
  assert(tvec.rows == 3);
  assert(rvec.cols == 1);
  assert(rvec.rows == 3);

//  if(tvec.type() != CV_32FC1)
//  {
//	tvec.convertTo(tvec,CV_32FC1);
//  }
//
//  if(rvec.type() != CV_32FC1)
//  {
//	rvec.convertTo(rvec,CV_32FC1);
//  }

  cv::Mat color;
  auto img = edgeImage->getImg();
  cv::cvtColor(img, color, cv::COLOR_GRAY2BGR);

  cv::Rect frame(cv::Point(0, 0), img.size());
  double CC = 0;
  auto transform = edgePointCloud;

  tvec.setTo(cv::Scalar(0));
  rvec.setTo(cv::Scalar(0));

  cout << "tvec" << tvec << "rvec" << rvec << endl;

  for (auto pt : transform) {

    Mat imagePoints;
    Point3f p3f;
    p3f.x = pt.x;
    p3f.y = pt.y;
    p3f.z = pt.z;

    vector<Point3f> objectPoints;
    objectPoints.push_back(p3f);
    cv::projectPoints(objectPoints, rvec, tvec, camera->K, Mat(), imagePoints);
    cv::Point xy = imagePoints.at<cv::Point2i>(1);

    if (pt.z > 0 && xy.inside(frame)) {

      assert(pt.intensity <= 1);
      CC += img.at<cv::Vec3b>(xy)[0]
          * pt.intensity; // šedý obrázek rgb sou si rovny vezmem jeden kanál
      color.at<cv::Vec3b>(xy)[1] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 255_rgb_c : 0_rgb_c;
      color.at<cv::Vec3b>(xy)[2] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 0_rgb_c : 255_rgb_c;
      color.at<cv::Vec3b>(xy)[0] = static_cast<u_char>(pt.intensity * 255);
    }
  }
  SHOW_IMAGE(color, title);
  return CC;
}

double Calibrator::getSimilarity2(cv::Mat &tvec,
                                  cv::Mat &rvec,
                                  const char *title) {

  assert(tvec.cols == 1);
  assert(tvec.rows == 3);
  assert(rvec.cols == 1);
  assert(rvec.rows == 3);

  if (tvec.type() != CV_64FC1) {
    tvec.convertTo(tvec, CV_64FC1);
  }

  if (rvec.type() != CV_64FC1) {
    rvec.convertTo(rvec, CV_64FC1);
  }

  cv::Mat color;
  auto img = edgeImage->getImg();
  cv::cvtColor(img, color, cv::COLOR_GRAY2BGR);

  cv::Rect frame(cv::Point(0, 0), img.size());
  double CC = 0;
  auto transform = edgePointCloud.transform(tvec, rvec);
  cout << "tvec" << tvec << endl;

  for (auto pt : transform) {

    cv::Point xy = Velodyne::Velodyne::project(pt, camera->P);
    //cv::Point xy = camera->project(Point3f(pt.x, pt.y, pt.z));

    if (pt.z > 0 && xy.inside(frame)) {

      assert(pt.intensity <= 1);
      CC += img.at<cv::Vec3b>(xy)[0]
          * pt.intensity; // šedý obrázek rgb sou si rovny vezmem jeden kanál
      color.at<cv::Vec3b>(xy)[1] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 255_rgb_c : 0_rgb_c;
      color.at<cv::Vec3b>(xy)[2] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 0_rgb_c : 255_rgb_c;
      color.at<cv::Vec3b>(xy)[0] = static_cast<u_char>(pt.intensity * 255);
    }
  }
  SHOW_IMAGE(color, title);
  return CC;
}

Calibrator::~Calibrator() {
  delete edgeImage;
}

Calibration6DoF Calibrator::findTranslation(std::vector<cv::Point2f> image,
                                            std::vector<cv::Point3f> velodyne,
                                            double radius2D,
                                            double radius3D) {
  auto projection = camera->P;

  Mat rvec(3, 1, CV_32FC1);
  rvec.at<double>(0, 0) = 0;
  rvec.at<double>(1, 0) = 0;
  rvec.at<double>(2, 0) = 0;

  Mat tvec(3, 1, CV_32FC1);

  double focal_len = projection.at<double>(0, 0);

  // t_z:
  tvec.at<double>(Z) = radius3D * focal_len / radius2D - velodyne.front().z;

  double principal_x = projection.at<double>(0, 2);
  double principal_y = projection.at<double>(1, 2);

  for (size_t i = 0; i < image.size(); i++) {
    // t_x:
    tvec.at<double>(X) +=
        (image[i].x - principal_x) * (velodyne[i].z + tvec.at<double>(Z))
            / focal_len
            - velodyne[i].x;
    // t_y:
    tvec.at<double>(Y) +=
        (image[i].y - principal_y) * (velodyne[i].z + tvec.at<double>(Z))
            / focal_len
            - velodyne[i].y;
  }
  tvec.at<double>(X) /= image.size();
  tvec.at<double>(Y) /= image.size();



  // no rotation and value of calibration
  return Calibration6DoF(tvec,
                         rvec,
                         getSimilarity2(tvec, rvec, "findTranslation"));
}

void Calibrator::calibrationRefinement(
    Calibration6DoF &rough,
    float max_translation,
    float max_rotation,
    unsigned steps,
    Calibration6DoF &best_calibration,
    Calibration6DoF &average
) {

  float x_rough = static_cast<float>(rough.tX());
  float y_rough = static_cast<float>(rough.tY());
  float z_rough = static_cast<float>(rough.tZ());

  auto P = camera->P;

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

  Velodyne::Velodyne transformed =
      edgePointCloud.transform(x_rough, y_rough, z_rough, 0, 0, 0);
  double rough_val = Similarity::edgeSimilarity(image, transformed, P);

  best_calibration.set(static_cast<double>(x_rough), static_cast<double>(y_rough), static_cast<double>(z_rough), 0, 0, 0, rough_val);

  int counter = 0;

  float x = x_min;
  for (size_t xi = 0; xi < steps; xi++) {

    float y = y_min;
    for (size_t yi = 0; yi < steps; yi++) {

      float z = z_min;
      for (size_t zi = 0; zi < steps; zi++) {

        float x_r = x_rot_min;
        for (size_t x_ri = 0; x_ri < steps; x_ri++) {

          float y_r = y_rot_min;
          for (size_t y_ri = 0; y_ri < steps; y_ri++) {

            float z_r = z_rot_min;
            for (size_t z_ri = 0; z_ri < steps; z_ri++) {

              transformed = edgePointCloud.transform(x, y, z, x_r, y_r, z_r);
              double value = Similarity::edgeSimilarity(image, transformed, P);
              Calibration6DoF calibration(
                  static_cast<double>(x),
                  static_cast<double>(y),
                  static_cast<double>(z),
                  static_cast<double>(x_r),
                  static_cast<double>(y_r),
                  static_cast<double>(z_r),
                  value
              );
              if (value > best_calibration.value) {
                best_calibration.set(
                    static_cast<double>(x),
                    static_cast<double>(y),
                    static_cast<double>(z),
                    static_cast<double>(x_r),
                    static_cast<double>(y_r),
                    static_cast<double>(z_r),
                    value
                );
              }

              if (value > rough_val) {
                average += calibration;
                counter++;
              }
              cout << counter << ".\t" << std::flush;
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

