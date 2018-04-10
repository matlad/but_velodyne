/**
 * @file     Colorizer.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_COLORIZER_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_COLORIZER_H

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "camera_info_manager/camera_info_manager.h"
#include "opencv2/core/mat.hpp"
#include "pcl/point_types.h"
#include "Velodyne.h"

namespace But::calibration_camera_velodyne {
/**
 * @brief Třída pro obarvování pointcoudu
 */
class Colorizer {

  ros::Publisher publisher;

  /**
   * @brief obraz pomocí něho bude pointcloud obarven
   */
  cv::Mat frame_rgb;

  /**
   * @brief External camera parameters
   */
  std::vector<float> DoF;

  /**
   * @brief Internal camera parameters
   */
  cv::Mat projection_matrix;

  cv::Mat D;

  cv::Mat K;

  But::calibration_camera_velodyne::Velodyne::Velodyne pointCloud;

 public:
  Colorizer(const ros::Publisher &publisher,
			const std::vector<float> &DoF);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  pcl::PointCloud <pcl::PointXYZRGB>colourByFishEye(cv::Mat frame_rgb,
						  cv::Mat D,
						  cv::Mat K,
						  cv::Mat rvec,
						  cv::Mat tvec);
};
}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_COLORIZER_H
