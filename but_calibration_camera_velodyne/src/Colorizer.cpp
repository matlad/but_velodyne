/**
 * @file     Colorizer.cpp
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include <Eigen/Dense>
#include "but_calibration_camera_velodyne/Colorizer.h"

#include <cstdlib>
#include <cstdio>

#include "opencv2/opencv.hpp"

#include <tf/tf.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <but_calibration_camera_velodyne/Image.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/macros.h>
#include <but_calibration_camera_velodyne/Calibrator.h>
#include <but_calibration_camera_velodyne/types.h>
#include <but_calibration_camera_velodyne/Constants.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace but::calibration_camera_velodyne;
using namespace but::calibration_camera_velodyne::velodyne;

void Colorizer::setFrontImage(Mat &image) {
  frontImage = image;
  cv::rotate(frontImage, frontImage, cv::ROTATE_90_CLOCKWISE);
  auto img = image::Image(frontImage);
  auto edge = img.computeEdgeImage();
  cv::cvtColor(edge, frontImage, cv::COLOR_GRAY2BGR);
  //SHOW_IMAGE(frontImage,"frontImage");
}

void Colorizer::setBackImage(Mat &image) {
  backImage = image;
  cv::rotate(backImage, backImage, cv::ROTATE_90_COUNTERCLOCKWISE);
  auto img = image::Image(backImage);
  auto edge = img.computeEdgeImage();
  cv::cvtColor(edge, backImage, cv::COLOR_GRAY2BGR);
  //SHOW_IMAGE(backImage,"backImage");
}

void Colorizer::setFrontCamera(CameraPtr camera) {
  frontCamera = camera;
}

void Colorizer::setBackCamera(CameraPtr camera) {
  backCamera = camera;
}

void Colorizer::setPointCloud(velodyne::Velodyne pointCloud) {
  this->pointCloud = pointCloud.ros2ButCoordinateSystem();
  //this->pointCloud.view();
}

PointCloud<PointXYZRGB> Colorizer::colorize() {

  auto colorCloud = colourByFishEye();

  // reverse axix switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  transformPointCloud(colorCloud, colorCloud, transf);

  return colorCloud;
}

PointCloud<PointXYZRGB> Colorizer::colourByFishEye() {
  
  DEBUG_STREAM(*frontCamera);

  auto pointCloudTransform = pointCloud.transform(frontCamera->tvec, frontCamera->rvec);

  vector<Point3f> cvPointCloud;
  pointCloudTransform.convertTo(&cvPointCloud);


  Mat zeroVec = VEC_3D;
  Mat back = VEC_3D;

  vector<Point2f> frontProjectedPoints;

  fisheye::projectPoints(
      cvPointCloud,
      frontProjectedPoints,
      zeroVec,
      zeroVec,
      frontCamera->K,
      frontCamera->D
  );

  Vec3b rgbDefault(255, 0, 0);

  cv::Rect frame(cv::Point(0, 0), frontImage.size());

  for (size_t iter = 0; iter < pointCloud.size(); iter++) {

    assert(iter <= INT_MAX);

    Point2f xy = frontProjectedPoints[iter];
    Vec3b rgb;

    if (xy.inside(frame)) {
      if (pointCloud[iter].z > 0) {
        rgb = image::Image::atf(frontImage, xy);
      } else {
        rgb = image::Image::atf(backImage, xy);
      }
    } else {
      rgb = rgbDefault;
    }

    PointXYZRGB pt_rgb(rgb.val[RED], rgb.val[GREEN], rgb.val[BLUE]);
    pt_rgb.x = pointCloud[iter].x;
    pt_rgb.y = pointCloud[iter].y;
    pt_rgb.z = pointCloud[iter].z;

    colorPointCloud.push_back(pt_rgb);
  }
  DEBUG_STREAM(colorPointCloud);
  return colorPointCloud;
}
