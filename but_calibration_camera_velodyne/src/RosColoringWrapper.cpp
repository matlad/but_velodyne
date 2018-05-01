/**
 * @file     RosColoringWrapper.cpp
 * @encoding UTF-8
 * @date     17.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */

#include <cv_bridge/cv_bridge.h>
#include <but_calibration_camera_velodyne/FishEyeCamera.h>
#include <pcl_conversions/pcl_conversions.h>
#include <but_calibration_camera_velodyne/RosColoringWrapper.h>

namespace but::calibration_camera_velodyne {
using pcl::PointCloud;
using pcl::fromROSMsg;
using pcl::toROSMsg;
using cv::ROTATE_90_COUNTERCLOCKWISE;

void RosColoringWrapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &imageInfo) {
  auto camera = FishEyeCamera();
  cv::Mat(3, 4, CV_64FC1, (void *) &imageInfo->P).copyTo(camera.P);
  camera.P.convertTo(camera.P, CV_32FC1);
  cv::Mat(imageInfo->D).copyTo(camera.D);
  rotate(camera.D, camera.D, ROTATE_90_COUNTERCLOCKWISE);
  cv::Mat(3, 3, CV_64FC1, (void *) &imageInfo->K).copyTo(camera.K);

  camera.tvec = cv::Mat(1, 3, CV_64F, sixDoF.data());
  camera.rvec = cv::Mat(1, 3, CV_64F, sixDoF.data() + 3);

  ROS_DEBUG_STREAM("P: \n" << camera.P);
  ROS_DEBUG_STREAM("D: \n" << camera.D);
  ROS_DEBUG_STREAM("K: \n" << camera.K);

  colorizer.setCamera(&camera);
}

void RosColoringWrapper::imageFrameCallback(const sensor_msgs::ImageConstPtr &image) {
  boost::shared_ptr<cv_bridge::CvImage>
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  colorizer.setImage(cv_ptr->image);
}

void RosColoringWrapper::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pointCloud) {
  // Loading Velodyne point cloud
  PointCloud<velodyne::Point> pc;
  fromROSMsg(*pointCloud, pc);

  colorizer.setPointCloud(pc);

  auto colorCloud = colorizer.colorize();

  sensor_msgs::PointCloud2 rosColorCloud;
  toROSMsg(colorCloud, rosColorCloud);
  rosColorCloud.header = pointCloud->header;
  publisher.publish(rosColorCloud);
}

RosColoringWrapper::RosColoringWrapper(
    const ros::Publisher &publisher,
    const std::vector<double> &sixDoF)
    : publisher(publisher), sixDoF(sixDoF) {}
}
