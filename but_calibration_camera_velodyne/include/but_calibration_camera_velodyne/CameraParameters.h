/**
 * @file     CameraParamerets.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_CAMERAPARAMERETS_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_CAMERAPARAMERETS_H

#include <opencv2/core/matx.hpp>
#include <opencv2/core/mat.hpp>
namespace But::calibration_camera_velodyne {

class CameraParameters {
 public:

  /**
   * @brief Kamera matrix
   */
  cv::Mat K;

  /**
   * @brief Projection matrix
   */
  cv::Mat P;

  /**
   * @brief koeficienty zkreslení
   */
  cv::Mat D;

  cv::Vec3f tvec;

  cv::Vec3f rvec;
};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CAMERAPARAMERETS_H
