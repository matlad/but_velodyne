/**
 * @file     fishEyeCamera.cpp
 * @encoding UTF-8
 * @date     10.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include "but_calibration_camera_velodyne/FishEyeCamera.h"

namespace but::calibration_camera_velodyne {
using cv::Point2i;
using cv::Point3f;
using cv::Mat;
using std::vector;
using cv::fisheye::projectPoints;

Point2i FishEyeCamera::project(Point3f) {
  Mat imagePoints;
  vector<Point3f> objectPoints;

  cv::fisheye::projectPoints(objectPoints, imagePoints, rvec, tvec, K, D);
  return imagePoints.at<Point2i>(1);
}

image::Image FishEyeCamera::undisort(image::Image image) {
  auto img = image.getImg();
  cv::fisheye::undistortImage(img, img, K, D, K);
  return image::Image(img);
}

}
