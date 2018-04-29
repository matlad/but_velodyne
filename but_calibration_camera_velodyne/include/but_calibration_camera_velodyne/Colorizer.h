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
#include "FishEyeCamera.h"

namespace but::calibration_camera_velodyne {
/**
 * @brief Třída pro obarvování pointcoudu
 */
class Colorizer {

  /**
   * @brief obraz pomocí něho bude pointcloud obarven
   */
  cv::Mat image;

  Camera * camera;

  but::calibration_camera_velodyne::Velodyne::Velodyne pointCloud;

 public:

  void setImage(cv::Mat& image);

  void setCamera(Camera * camera);

  void setPointCloud(but::calibration_camera_velodyne::Velodyne::Velodyne pointCloud);

  pcl::PointCloud <pcl::PointXYZRGB>colourByFishEye();

  pcl::PointCloud <pcl::PointXYZRGB> colorize();
};
}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_COLORIZER_H
