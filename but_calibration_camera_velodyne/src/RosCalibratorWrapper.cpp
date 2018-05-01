/**
 * @file     RosCalibratorWrapper.cpp
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
//#include <node.h>
#include <sys/socket.h>
#include <pcl/point_cloud.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <pcl_conversions/pcl_conversions.h>
#include <but_calibration_camera_velodyne/macros.h>
#include "but_calibration_camera_velodyne/RosCalibratorWrapper.h"
#include "but_calibration_camera_velodyne/FishEyeCamera.h"
#include "but_calibration_camera_velodyne/Calibration6DoF.h"

#define BREAK 0

using pcl::PointCloud;
using pcl::fromROSMsg;
using cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;
using namespace but::calibration_camera_velodyne;
using ros::shutdown;

void RosCalibratorWrapper::callback(
    const sensor_msgs::ImageConstPtr &msg_img,
    const sensor_msgs::CameraInfoConstPtr &msg_info,
    const sensor_msgs::PointCloud2ConstPtr &msg_pc
) {

  ROS_DEBUG_STREAM("Image received at " << msg_img->header.stamp.toSec());
  ROS_DEBUG_STREAM(
      "Camera info received at " << msg_info->header.stamp.toSec());
  ROS_DEBUG_STREAM(
      "Velodyne scan received at " << msg_pc->header.stamp.toSec());

  processImageInfo(msg_info);
  processImage(msg_img);
  processPointCloud(msg_pc);

  // calibration:
  Calibration6DoF calibrationParams = calibrator.calibration(true);
  if (calibrationParams.isGood()) {
    ROS_INFO_STREAM("Calibration succeeded, found parameters:");
    calibrationParams.print();
    shutdown();
  } else {
    ROS_WARN("Calibration failed - trying again after "
                 BUT_STR(BREAK)
                 "s ...");
    ros::Duration(BREAK).sleep();
  }
}

void RosCalibratorWrapper::processImageInfo(const sensor_msgs::CameraInfoConstPtr &imageInfo) {
  auto camera = FishEyeCamera();
  cv::Mat(3, 4, CV_64FC1, (void *) &imageInfo->P).copyTo(camera.P);
  camera.P.convertTo(camera.P, CV_32FC1);
  cv::Mat(imageInfo->D).copyTo(camera.D);
  rotate(camera.D, camera.D, ROTATE_90_COUNTERCLOCKWISE);
  cv::Mat(3, 3, CV_64FC1, (void *) &imageInfo->K).copyTo(camera.K);

  ROS_DEBUG_STREAM("P: \n" << camera.P);
  ROS_DEBUG_STREAM("D: \n" << camera.D);
  ROS_DEBUG_STREAM("K: \n" << camera.K);

  calibrator.setCamera(&camera);
}

void RosCalibratorWrapper::processImage(const sensor_msgs::ImageConstPtr &image) {
  boost::shared_ptr<cv_bridge::CvImage>
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  calibrator.setImage(cv_ptr->image);
}

void RosCalibratorWrapper::processPointCloud(const sensor_msgs::PointCloud2ConstPtr &pointCloud) {
  // Loading Velodyne point cloud
  PointCloud<velodyne::Point> pc;
  fromROSMsg(*pointCloud, pc);
  calibrator.setPointCloud(pc);
}

RosCalibratorWrapper::RosCalibratorWrapper(double distance, double radius) :
    calibrator(distance, radius) {}



