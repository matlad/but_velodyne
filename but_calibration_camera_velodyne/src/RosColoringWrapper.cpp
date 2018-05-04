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
#include <but_calibration_camera_velodyne/macros.h>

namespace but::calibration_camera_velodyne {
using pcl::PointCloud;
using pcl::fromROSMsg;
using pcl::toROSMsg;
using cv::ROTATE_90_COUNTERCLOCKWISE;

void RosColoringWrapper::frontCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &imageInfo
) {
  CameraPtr camera = imageInfoToCamera(imageInfo);
  colorizer.setFrontCamera(camera);
}

void RosColoringWrapper::backCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &imageInfo
) {
  CameraPtr camera = imageInfoToCamera(imageInfo);
  colorizer.setBackCamera(camera);
}

CameraPtr RosColoringWrapper::imageInfoToCamera(
    const sensor_msgs::CameraInfoConstPtr &imageInfo
) const {
  CameraPtr camera = std::make_shared<FishEyeCamera>();
  cv::Mat(3, 4, CV_64FC1, (void *) &imageInfo->P).copyTo(camera->P);
  camera->P.convertTo(camera->P, CV_32FC1);
  cv::Mat(imageInfo->D).copyTo(camera->D);
  rotate(camera->D, camera->D, ROTATE_90_COUNTERCLOCKWISE);
  cv::Mat(3, 3, CV_64FC1, (void *) &imageInfo->K).copyTo(camera->K);



  camera->tvec = cv::Mat(3, 1, CV_64F, (void *) sixDoF.data());
  camera->rvec = cv::Mat(3, 1, CV_64F, (void *) (sixDoF.data() + 3));

  //DEBUG_STREAM(camera);
  return camera;
}

void RosColoringWrapper::frontImageFrameCallback(const sensor_msgs::ImageConstPtr &image) {
  boost::shared_ptr<cv_bridge::CvImage> cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  colorizer.setFrontImage(cv_ptr->image);
}

void RosColoringWrapper::backImageFrameCallback(const sensor_msgs::ImageConstPtr &image) {
  boost::shared_ptr<cv_bridge::CvImage> cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  colorizer.setBackImage(cv_ptr->image);
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
