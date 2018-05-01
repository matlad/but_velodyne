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
#include <boost/foreach.hpp>

#include "opencv2/opencv.hpp"

#include <image_transport/image_transport.h>
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

void Colorizer::setImage(Mat &image) {
  this->image = image;
}

void Colorizer::setCamera(CameraPtr camera) {
  this->camera = camera;
}

void Colorizer::setPointCloud(velodyne::Velodyne pointCloud) {
  ;
  this->pointCloud = pointCloud.ros2ButCoordinateSystem();
  //this->pointCloud.view();
}

PointCloud<PointXYZRGB> Colorizer::colorize() {

  auto colorCloud = colourByFishEye();

  // reverse axix switching:
  Eigen::Affine3f transf = getTransformation(0, 0, 0, -M_PI / 2, 0, 0);
  // transformPointCloud(colorCloud, colorCloud, transf);

  return colorCloud;
}

PointCloud<PointXYZRGB> Colorizer::colourByFishEye() {

  clog << *camera << endl;

  PointCloud<PointXYZRGB> colorPointCloud;

  auto pointCloudTransform = pointCloud.transform(camera->tvec, camera->rvec);

  Mat color;
  Mat image;
  fisheye::undistortImage(this->image, color, camera->K, camera->D, camera->K);

  cv::rotate(color, color, cv::ROTATE_90_CLOCKWISE);
  //SHOW_IMAGE(color,"undisort");


  cv::Rect frame(cv::Point(0, 0), color.size());
  pointCloudTransform.normalizeIntensity(0, 1);
  //pointCloudTransform.view(0, "pointCloudTransform");
  for (auto pt : pointCloudTransform) {

    cv::Point xy = velodyne::Velodyne::project(pt, camera->P);
    //cv::Point xy = camera->project(Point3f(pt.x, pt.y, pt.z));

    if (pt.z > 0 && xy.inside(frame)) {

      assert(pt.intensity <= 1);

      // Překreslení bodů z kamery projekcí bodu z lidaru
      color.at<cv::Vec3b>(xy)[RED] = static_cast<u_char>(pt.intensity * 255);
      color.at<cv::Vec3b>(xy)[GREEN] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 255_rgb_c : 0_rgb_c;
      color.at<cv::Vec3b>(xy)[BLUE] =
          pt.intensity < POINTCLOUD_EDGE_TRASH_HOLD ? 0_rgb_c : 255_rgb_c;
    }
  }
  SHOW_IMAGE(color, "simlrarity");

  cv::rotate(this->image, image, cv::ROTATE_90_CLOCKWISE);

  //pointCloud = pointCloud.transform(camera->tvec, camera->rvec);

  vector<Point3f> cvPointCloud;
  auto pclPointCloud = this->pointCloud.getPointCloud();

  for (auto point:pointCloudTransform) {
    cvPointCloud.push_back(Point3d((double) point.x,
                                   (double) point.y,
                                   (double) point.z));
  }

  Mat zeroVec = VEC_3D;

  vector<Point2f> projectedPoints;

  fisheye::projectPoints(
      cvPointCloud,
      projectedPoints,
      zeroVec,
      zeroVec,
      camera->K,
      camera->D
  );

  cout << *camera << endl;

  Vec3b rgbDefault(255, 0, 0);

  for (size_t iter = 0; iter < pclPointCloud.points.size(); iter++) {

    assert(iter <= INT_MAX);

    Point2f xy = projectedPoints[iter];
    Vec3b rgb;

    if (pclPointCloud.at(iter).z < 0 || xy.y < 0 || xy.x < 0
        || xy.y > image.rows || xy.x > image.cols) {
      rgb = rgbDefault;
    } else {
      rgb = image::Image::atf(image, xy);
    }

    PointXYZRGB pt_rgb(rgb.val[RED], rgb.val[GREEN], rgb.val[BLUE]);
    pt_rgb.x = pclPointCloud.at(iter).x;
    pt_rgb.y = pclPointCloud.at(iter).y;
    pt_rgb.z = pclPointCloud.at(iter).z;

    colorPointCloud.push_back(pt_rgb);
  }
  return colorPointCloud;
}
