/**
 * @file     RosColoringWrapper.h
 * @encoding UTF-8
 * @date     17.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_ROSCOLORINGWRAPPER_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_ROSCOLORINGWRAPPER_H

#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "but_calibration_camera_velodyne/Colorizer.h"

namespace but::calibration_camera_velodyne {

class RosColoringWrapper {

  std::vector<double> sixDoF;
  Colorizer colorizer;
  ros::Publisher publisher;

 public:
  RosColoringWrapper(const ros::Publisher &publisher,const std::vector<float> &sixDoF);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &imageInfo);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud);
  void imageFrameCallback(const sensor_msgs::ImageConstPtr &image);

};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_ROSCOLORINGWRAPPER_H
