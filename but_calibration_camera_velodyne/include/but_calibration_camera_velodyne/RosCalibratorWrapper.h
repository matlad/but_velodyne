/**
 * @file     RosCalibratorWrapper.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_ROSCALIBRATORWRAPPER_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_ROSCALIBRATORWRAPPER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "opencv2/opencv.hpp"
#include "but_calibration_camera_velodyne/Calibrator.h"

class RosCalibratorWrapper {
  but::calibration_camera_velodyne::Calibrator calibrator;

 public:
  void callback(
      const sensor_msgs::ImageConstPtr &frontImgMsg,
      const sensor_msgs::CameraInfoConstPtr &frontInfoMsg,
      const sensor_msgs::ImageConstPtr &backImgMsg,
      const sensor_msgs::CameraInfoConstPtr &backInfoMsg,
      const sensor_msgs::PointCloud2ConstPtr &msg_pc
  );
  void processFrontImageInfo(const sensor_msgs::CameraInfoConstPtr &imageInfo);
  void processFrontImage(const sensor_msgs::ImageConstPtr &image);
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr &pointCloud);

  RosCalibratorWrapper(double distance, double radius);
  but::calibration_camera_velodyne::CameraPtr imageInfoToCameraPtr(const sensor_msgs::CameraInfoConstPtr &imageInfo) const;
  void processBackImageInfo(const sensor_msgs::CameraInfoConstPtr &imageInfo);
  void processBackImage(const sensor_msgs::ImageConstPtr &image);
};

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_ROSCALIBRATORWRAPPER_H
