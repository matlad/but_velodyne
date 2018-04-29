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
#include "but_calibration_camera_velodyne/Image.h"

namespace but::calibration_camera_velodyne {

class Camera {
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

  cv::Mat tvec;

  cv::Mat rvec;

  /**
   * @brief
   * @return
   */
  virtual Image::Image undisort(Image::Image) = 0;

  /**
   * @brief Promítne
   * @return
   */
  virtual cv::Point2i project(cv::Point3f) = 0;
};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CAMERAPARAMERETS_H
