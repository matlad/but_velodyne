/**
 * @file     Camera.cpp
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include "but_calibration_camera_velodyne/Camera.h"

namespace but::calibration_camera_velodyne {

double Camera::K_fx() {
  return K.at<double>(0, 0);
}

double Camera::K_fy() {
  return K.at<double>(1, 1);
}

double Camera::K_cx() {
  return K.at<double>(0, 2);
}

double Camera::K_cy() {
  return K.at<double>(1, 2);
}

}

std::ostream &but::calibration_camera_velodyne::operator<<(
    std::ostream &os,
    const but::calibration_camera_velodyne::Camera &camera
) {
  os <<
     "\n" <<
     "--Camera parameters:------------------------------------------------------------\n"
     <<
     "K: " << camera.K << "\n\n" <<
     "P: " << camera.P << "\n\n" <<
     "D: " << camera.D << "\n\n" <<
     "tvec: " << camera.tvec << "\n\n" <<
     "rvec: " << camera.rvec << "\n\n" <<
     "--------------------------------------------------------------------------------\n";
  return os;
}
