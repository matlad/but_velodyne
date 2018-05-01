/**
 * @file     fishEyeCamera.h
 * @encoding UTF-8
 * @date     10.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_FISHEYECAMERA_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_FISHEYECAMERA_H

#include "but_calibration_camera_velodyne/Camera.h"
#include "but_calibration_camera_velodyne/Image.h"

namespace but::calibration_camera_velodyne {

class FishEyeCamera : public Camera {
 public:

  /**
   * @brief
   * @return
   */
  image::Image undisort(image::Image);

  /**
   * @brief Promítne
   * @return
   */
  cv::Point2i project(cv::Point3f);

};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_FISHEYECAMERA_H
