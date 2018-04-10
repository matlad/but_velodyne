/**
 * @file     Calibrator.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H

#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/CameraParameters.h>
#include <but_calibration_camera_velodyne/Calibration3DMarker.h>
#include "Calibration6DoF.h"

namespace But::calibration_camera_velodyne {

class Calibrator {

  /**
   * @brief Zpracovávaný obraz
   */
  cv::Mat image;

  /**
   *  Obraz, který byl převeden na odstíny šedi a byla
   *  na něm provedeno zvíraznění hran.
   */
  Image::Image *edgeImage = nullptr;

  /**
   * @brief Zpracovávaný point cloud
   */
  Velodyne::Velodyne pointCloud;

  /**
   * @brief pointCloud s intensity podle hran
   */
  Velodyne::Velodyne edgePointCloud;

  /**
   * @brief Vnitřní parametry kamery
   */
  CameraParameters cameraParameters;

  /**
   * @brief Vzdálenost kruhů v
   */
  float circleDistance;

  /**
   * @brief Rádius kruhů markeru
   */
  float radius;

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
  void setPointCloud(Velodyne::VPointCloud cloud);

  /**
   * @brief nastaví parametry kamery
   * @param cameraParameters
   */
  void setCameraParameters(CameraParameters &cameraParameters);

  /**
   * @brief opraví zkreslení zpracovávaného obrazu
   */
  void undisortImage();

  /**
   * @brief Odhadne translaci kamery vůči lidaru
   * Předpokádá, že kamera a lidar jsou natočeny teměř stejně a to čelem kolmo k markeru
   * @param image středy kruhů v obraze
   * @param velodyne středy kruhů v pointCloudu
   * @param radius2D  radius kruhů v obraze
   * @param radius3D  rádius kruhů v poitCloudu
   * @return
   */
  Calibration6DoF findTranslation(std::vector<cv::Point2f> image,
								  std::vector<cv::Point3f> velodyne,
								  float radius2D,
								  float radius3D);

  /**
   * @brief Doladění externách parametrů
   * Jde o brutal Force vezme se výchozí pozice a upravují se jednotlivé parametry o steps
   * v každém kroku je vypočítaná krosreference hran v obraze a pointCloudu (@see Similarity::edgeSimilarity)
   * a hledá se, která poloha má tuto hodnotu nejvyšší;
   */
  void calibrationRefinement(Calibration6DoF &rough,
							 float max_translation,
							 float max_rotation,
							 unsigned steps,
							 Calibration6DoF &best_calibration,
							 Calibration6DoF &average);

  Calibration6DoF calibration(bool doRefinement);

  Calibrator(float circleDistance, float radius);

  /**
 * @brief provede reprojekci a spočítá podobnost
 * @param tvec
 * @param rvec
 * @param z
 * @param x_r
 * @param y_r
 * @param z_r
 */
  float getSimilarity(cv::Mat &tvec, cv::Mat &rvec);

  virtual ~Calibrator();


};

}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATOR_H
