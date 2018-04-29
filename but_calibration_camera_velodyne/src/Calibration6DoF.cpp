/**
 * @file     Calibration6DoF.cpp
 * @encoding UTF-8
 * @date     1.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include <but_calibration_camera_velodyne/Constants.h>
#include "but_calibration_camera_velodyne/Calibration6DoF.h"

using namespace but::calibration_camera_velodyne;

double Calibration6DoF::tX() {
  return DoF[0];
}

double Calibration6DoF::tY() {
  return DoF[1];
}

double Calibration6DoF::tZ() {
  return DoF[2];
}

double Calibration6DoF::rX() {
  return DoF[3];
}

double Calibration6DoF::rY() {
  return DoF[4];
}

double Calibration6DoF::rZ() {
  return DoF[5];
}

void Calibration6DoF::set(cv::Mat tvec, cv::Mat rvec, double val) {
  ASSERT_IS_VEC_3D(tvec);
  ASSERT_IS_VEC_3D(rvec);

  set(
	  tvec.at<double>(X), tvec.at<double>(Y), tvec.at<double>(Z),
	  rvec.at<double>(X), rvec.at<double>(Y), rvec.at<double>(Z),
	  val
  );
}
