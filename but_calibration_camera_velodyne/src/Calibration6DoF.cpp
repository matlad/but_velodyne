/**
 * @file     Calibration6DoF.cpp
 * @encoding UTF-8
 * @date     1.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include "but_calibration_camera_velodyne/Calibration6DoF.h"

using namespace But::calibration_camera_velodyne;

float Calibration6DoF::tX() {
  return DoF[0];
}

float Calibration6DoF::tY() {
  return DoF[1];
}

float Calibration6DoF::tZ() {
  return DoF[2];
}

float Calibration6DoF::rX() {
  return DoF[3];
}

float Calibration6DoF::rY() {
  return DoF[4];
}

float Calibration6DoF::rZ() {
  return DoF[5];
}

void Calibration6DoF::set(cv::Mat tvec, cv::Mat rvec, float val) {
  assert(tvec.cols == 1); assert(tvec.rows == 3);
  assert(rvec.cols == 1); assert(rvec.rows == 3);
  set(
	  tvec.at<float>(0, 0), tvec.at<float>(0, 1), tvec.at<float>(0, 2),
	  rvec.at<float>(0, 0), rvec.at<float>(0, 1), rvec.at<float>(0, 2),
	  val
  );
}
