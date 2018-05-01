/**
 * @file     Calibrator.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H

#define POINTCLOUD_EDGE_TRASH_HOLD 0.005

#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/Camera.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include "Calibration6DoF.h"
#include "but_calibration_camera_velodyne/Camera.h"

namespace but::calibration_camera_velodyne {

class Calibrator {

  /**
   * @brief Zpracovávaný obraz
   */
  cv::Mat image;

  /**
   *  Obraz, který byl převeden na odstíny šedi a byla
   *  na něm provedeno zvíraznění hran.
   */
  image::Image *edgeImage = nullptr;

  /**
   * @brief Zpracovávaný point cloud
   */
  velodyne::Velodyne pointCloud;

  /**
   * @brief pointCloud s intensity podle hran
   */
  velodyne::Velodyne edgePointCloud;

  /**
   * @brief Vnitřní parametry kamery
   */
  Camera *camera;

  /**
   * @brief Vzdálenost kruhů v
   */
  double circleDistance;

  /**
   * @brief Rádius kruhů markeru
   */
  double radius;

 public:

  /**
   * @brief Nastaví obraz pro zpracování.
   * @waring cameraParameters musí bít nastaveny před voláním této fce.
   * @param image
   */
  void setImage(cv::Mat image);

  /**
   *  Nastaví pointcloud ke zpracování
   */
  void setPointCloud(velodyne::oldVPointCloud cloud);

  /**
   * @brief nastaví parametry kamery
   * @param camera
   */
  void setCamera(Camera *camera);

  /**
   * @brief opraví zkreslení zpracovávaného obrazu
   */
  void undistortImage();

  /**
   * @brief Odhadne translaci kamery vůči lidaru
   * Předpokádá, že kamera a lidar jsou natočeny teměř stejně a to čelem kolmo k markeru
   * @param image středy kruhů v obraze
   * @param velodyne středy kruhů v pointCloudu
   * @param radius2D  radius kruhů v obraze
   * @param radius3D  rádius kruhů v poitCloudu
   * @return
   */
  void findTranslation(
      std::vector<cv::Point2f> image,
      std::vector<cv::Point3f> velodyne,
      double radius2D,
      double radius3D,
      cv::Mat rVec,
      cv::Mat tVec
  );

  /**
   * @brief Doladění externách parametrů
   * Jde o brutal Force vezme se výchozí pozice a upravují se jednotlivé parametry o steps
   * v každém kroku je vypočítaná krosreference hran v obraze a pointCloudu (@see Similarity::edgeSimilarity)
   * a hledá se, která poloha má tuto hodnotu nejvyšší;
   */
  void calibrationRefinement(
      cv::Mat tVec,
      cv::Mat rVec,
      float max_translation,
      float max_rotation,
      unsigned steps,
      Calibration6DoF &best_calibration,
      Calibration6DoF &average
  );

  Calibration6DoF calibration(bool doRefinement);

  Calibrator(double circleDistance, double radius);

  /**
 * @brief provede reprojekci a spočítá podobnost
 * @param tvec
 * @param rvec
 * @param z
 * @param x_r
 * @param y_r
 * @param z_r
 */
  double getSimilarity(
      cv::Mat &tvec,
      cv::Mat &rvec,
      const char *title = "getSimilarity"
  );

  virtual ~Calibrator();

  double getSimilarity2(cv::Mat &tvec, cv::Mat &rvec, const char *title);

  velodyne::Velodyne rawPCl;

  cv::Mat rawImg;
};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H
