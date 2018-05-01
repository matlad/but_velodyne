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
 * @brief Třída pro obarvování mračna bodů
 */
class Colorizer {

  /**
   * @brief Obraz pomocí něho bude mračno bodů obarveno.
   */
  cv::Mat image;

  /**
   * @brief Parametry, kamery pomoci které byl pořízen snímek k obarvení.
   */
  CameraPtr camera;

  /**
   * @brief Mračno bodů k obarvení k obarvení.
   */
  velodyne::Velodyne pointCloud;

 public:

  /**
   * @brief Nastaví obraz pomocí něhož se bude obarvovat.
   * @param image
   */
  void setImage(cv::Mat &image);

  /**
   * @brief Nastaví Parametry, kamery pomoci které byl pořízen snímek k obarvení.
   * @param camera
   */
  void setCamera(CameraPtr camera);

  /**
   * @brief Nastaví mračno bodů k obarvení.
   * @param pointCloud
   */
  void setPointCloud(velodyne::Velodyne pointCloud);

  /**
   * @brief Obarví mračno bodů.
   * @return Obarvené mračno
   */
  pcl::PointCloud<pcl::PointXYZRGB> colorize();

 private:

  /**
   * @brief Obarví mračno bodů snímkem z kamery typu rybí oko
   * @return Obarvené mračno
   */
  pcl::PointCloud<pcl::PointXYZRGB> colourByFishEye();
};
}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_COLORIZER_H
