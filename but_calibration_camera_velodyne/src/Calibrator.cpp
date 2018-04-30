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
using cv::Point3d;

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
  edgePointCloud.intensityByRangeDiff();
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

  Mat rVec = VEC_3D;
  Mat tVec = VEC_3D;

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
  pcl::PointCloud<pcl::PointXYZRGB> * colorCloud;

//##############################################################################
//
//  solvePnP(centers3D, centers2D, camera->K, Mat(), rVec, tVec);
//
//  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "solvePnP"));
//  cout << "solvePnP" << endl;
//  calib.print();
//
//  camera->rvec = rVec;
//  camera->tvec = tVec;
//  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
//  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

//##############################################################################

//  solvePnP(
//      centers3D,
//      centers2D,
//      camera->K,
//      Mat(),
//      rVec,
//      tVec,
//      false,
//      cv::SOLVEPNP_P3P
//  );
//
//  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "solvePnP - P3P"));
//  cout << "solvePnP - P3P" << endl;
//  calib.print();
//
//  camera->rvec = rVec;
//  camera->tvec = tVec;
//  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
//  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));


//##############################################################################


  solvePnP(centers3D,
           centers2D,
           camera->K,
           Mat(),
           rVec,
           tVec,
           false,
           cv::SOLVEPNP_AP3P);

  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "solvePnP - AP3P"));
  cout << "solvePnP - AP3P" << endl;
  calib.print();

  camera->rvec = rVec;
  camera->tvec = tVec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));


//##############################################################################

  double radius2D =
      accumulate(radii2D.begin(), radii2D.end(), 0.0) / radii2D.size();
  double radius3D =
      accumulate(radii3D.begin(), radii3D.end(), 0.0) / radii3D.size();

  // rough calibration
  findTranslation(centers2D, centers3D, radius2D, radius3D, rVec, tVec);
  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "findTranslation"));
  cout << "findTranslation" << endl;
  calib.print();

  camera->rvec = rVec;
  camera->tvec = tVec;
  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));


////##############################################################################
//
//  solvePnPRansac(centers3D,
//                 centers2D,
//                 camera->K,
//                 Mat(),
//                 rVec,
//                 tVec,
//                 true,
//                 100,
//                 0.8);
//
//  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "solvePnPRansac + findTranslation\""));
//  cout << "solvePnPRansac + findTranslation\"" << endl;
//  calib.print();
//
//  camera->rvec = rVec;
//  camera->tvec = tVec;
//  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
//  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));
//
////##############################################################################
//
//
//  solvePnPRansac(centers3D,
//                 centers2D,
//                 camera->K,
//                 Mat(),
//                 rVec,
//                 tVec,
//                 false,
//                 100,
//                 .8);
//
//  calib.set(tVec, rVec, getSimilarity(tVec, rVec, "solvePnPRansac"));
//  cout << "solvePnPRansac" << endl;
//  calib.print();
//
//  camera->rvec = rVec;
//  camera->tvec = tVec;
//  colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
//  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));
//
//
////##############################################################################
//
//
//  centers3D.pop_back();
//  centers2D.pop_back();
//
//
//  std::vector<Mat> rvecs, tvecs;
//
//  cv::solveP3P(centers3D,
//               centers2D,
//               camera->K,
//               Mat(),
//               rvecs,
//               tvecs,
//               cv::SOLVEPNP_P3P);
//
//  for (unsigned int i = 0; i < rvecs.size(); ++i) {
//    calib.set(tvecs[i], rvecs[i], getSimilarity(tvecs[i], rvecs[i], "solveP3P"));
//    cout << "solveP3P " << i << endl;
//    calib.print();
//  }
//
//
////##############################################################################
//
//
//  cv::solveP3P(centers3D,
//               centers2D,
//               camera->K,
//               Mat(),
//               rvecs,
//               tvecs,
//               cv::SOLVEPNP_AP3P);
//
//  for (unsigned int i = 0; i < rvecs.size(); ++i) {
//    calib.set(tvecs[i], rvecs[i], getSimilarity(tvecs[i], rvecs[i], "solveAP3P"));
//    cout << "solveAP3P " << i << endl;
//    calib.print();
//  }
//
//
////##############################################################################


  if (doRefinement) {
    u_int divisions = 5;
    float distance_transl = 0.02;
    float distance_rot = 0.01;
    Calibration6DoF best_calibration, avg_calibration;
    calibrationRefinement(tVec,
                          rVec,
                          distance_transl,
                          distance_rot,
                          divisions,
                          best_calibration,
                          avg_calibration);
    avg_calibration.print();

    calib.set(tVec, rVec, getSimilarity(tVec, rVec, "calibrationRefinement"));
    cout << "calibrationRefinement" << endl;
    calib.print();

    camera->rvec = rVec;
    camera->tvec = tVec;
    colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
    Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));
  }
    return calib;



}

Calibrator::Calibrator(double circleDistance, double radius) : circleDistance(
    circleDistance), radius(radius) {}

double Calibrator::getSimilarity(cv::Mat &tvec,
                                 cv::Mat &rvec,
                                 const char *title) {

  ASSERT_IS_VEC_3D(tvec);
  ASSERT_IS_VEC_3D(rvec);

  Mat color;
  auto img = edgeImage->getImg();
  cv::cvtColor(img, color, cv::COLOR_GRAY2BGR);

  cv::Rect frame(cv::Point(0, 0), img.size());
  double CC = 0;
  auto transform = edgePointCloud;

  Mat NoD = Mat(1, 5, CV_64FC1, cv::Scalar(0));

  for (auto pt : transform) {

    Mat imagePoints;
    Point3d p3d;
    p3d.x = (double)pt.x;
    p3d.y = (double)pt.y;
    p3d.z = (double)pt.z;

    vector<Point3d> objectPoints;
    objectPoints.push_back(p3d);
    cv::projectPoints(objectPoints, rvec, tvec, camera->K, NoD, imagePoints);
    cv::Point2d xy = imagePoints.at<cv::Point2d>(0);

    if (pt.z > 0 && xy.inside(frame)) {

      assert(pt.intensity <= 1);
      CC += img.at<cv::Vec3b>(xy)[RED]
          * pt.intensity; // šedý obrázek rgb sou si rovny vezmem jeden kanál
      color.at<cv::Vec3b>(xy)[BLUE] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 255_rgb_c : 0_rgb_c;
      color.at<cv::Vec3b>(xy)[GREEN] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 0_rgb_c : 255_rgb_c;
      color.at<cv::Vec3b>(xy)[RED] = static_cast<u_char>(pt.intensity * 255);
    }
  }
  SHOW_IMAGE(color, title);
  return CC;
}

Calibrator::~Calibrator() {
  delete edgeImage;
}

void Calibrator::findTranslation(std::vector<cv::Point2f> image,
                                 std::vector<cv::Point3f> velodyne,
                                 double radius2D,
                                 double radius3D,
                                 cv::Mat tVec,
                                 cv::Mat rVec) {
  auto projection = camera->P;

  rVec = VEC_3D;
  tVec = VEC_3D;

  double focal_len = camera->K_fx();

  // t_z:
  tVec.at<double>(Z) = radius3D * focal_len / radius2D - velodyne.front().z;

  double principal_x = camera->K_cx();
  double principal_y = camera->K_cy();

  for (size_t i = 0; i < image.size(); i++) {
    // t_x:
    tVec.at<double>(X) +=
        (image[i].x - principal_x) * (velodyne[i].z + tVec.at<double>(Z))
            / focal_len
            - velodyne[i].x;
    // t_y:
    tVec.at<double>(Y) +=
        (image[i].y - principal_y) * (velodyne[i].z + tVec.at<double>(Z))
            / focal_len
            - velodyne[i].y;
  }
  tVec.at<double>(X) /= image.size();
  tVec.at<double>(Y) /= image.size();
}

void Calibrator::calibrationRefinement(cv::Mat tVec,
                                       cv::Mat rVec,
                                       float max_translation,
                                       float max_rotation,
                                       unsigned steps,
                                       Calibration6DoF &best_calibration,
                                       Calibration6DoF &average) {

  IN_PROGRES_INIT();

  float x_rough = static_cast<float>(tVec.at<double>(X));
  float y_rough = static_cast<float>(tVec.at<double>(Y));
  float z_rough = static_cast<float>(tVec.at<double>(Z));

  float x_rough_r = static_cast<float>(rVec.at<double>(X));
  float y_rough_r = static_cast<float>(rVec.at<double>(Y));
  float z_rough_r = static_cast<float>(rVec.at<double>(Z));

  auto P = camera->P;

  auto image = Image::Image(this->image);
  image = Image::Image(image.computeIDTEdgeImage());

  float x_min = x_rough - max_translation;
  float y_min = y_rough - max_translation;
  float z_min = z_rough - max_translation;
  float x_rot_min = x_rough_r - max_rotation;
  float y_rot_min = y_rough_r - max_rotation;
  float z_rot_min = z_rough_r - max_rotation;

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

              IN_PROGRES_SPIN();

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

  tVec.at<double>(X) = best_calibration.tX();
  tVec.at<double>(Y) = best_calibration.tY();
  tVec.at<double>(Z) = best_calibration.tZ();
  rVec.at<double>(X) = best_calibration.rX();
  rVec.at<double>(Y) = best_calibration.rY();
  rVec.at<double>(Z) = best_calibration.rZ();
}

