/**
 * @file     calibration.cpp
 * @encoding UTF-8
 * @date     3.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 * @brief    Odhad pozice kamery vůči lidaru.
 */

#include <cstdlib>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>
#include <but_calibration_camera_velodyne/Colorizer.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/FishEyeCamera.h>
#include <but_calibration_camera_velodyne/Camera.h>

using cv::Mat;
using cv::imread;
using pcl::io::loadPCDFile;
using but::calibration_camera_velodyne::Colorizer;
using but::calibration_camera_velodyne::velodyne::oldVPointCloud;
using but::calibration_camera_velodyne::velodyne::Velodyne;
using but::calibration_camera_velodyne::FishEyeCamera;
using but::calibration_camera_velodyne::CameraPtr;
using std::cerr;
using std::endl;

int error(const char *msg) {
  cerr << msg << "\n"
       << " <frame> <camera-parameters> <point-cloud>"
       << endl;

  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {

  if (argc != 4) {
    return error("Chybný počet argumentů");
  }

  Mat image = imread(argv[1]);
  if (image.empty()) {
    return error("Chyba při načítání obrázku");
  }

  CameraPtr camera = std::shared_ptr<FishEyeCamera>();

  cv::FileStorage fs_P(argv[2], cv::FileStorage::READ);
  if (!fs_P.isOpened()) {
    return error("Chyba při otevření souboru s parametry kamery");
  }
  fs_P["P"] >> camera->P;
  fs_P["D"] >> camera->D;
  fs_P["K"] >> camera->K;
  fs_P["tvec"] >> camera->tvec;
  fs_P["rvec"] >> camera->rvec;
  fs_P.release();

  cout << "P:" << camera->P << "\n\n"
       << "D:" << camera->D << "\n\n"
       << "K:" << camera->K << "\n\n"
       << "tvec:" << camera->tvec << "\n\n"
       << "rvec:" << camera->rvec << "\n"
       << endl;

  oldVPointCloud pointCloud;
  pcl::io::loadPCDFile(argv[3], pointCloud);
  if (pointCloud.empty()) {
    return error("Chyba při načítání pointCloud");
  }

  Colorizer colorizer;
  colorizer.setCamera(camera);
  colorizer.setImage(image);
  colorizer.setPointCloud(pointCloud);

  auto colorCloud = new pcl::PointCloud<pcl::PointXYZRGB>(colorizer.colorize());
  Velodyne::Velodyne::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(colorCloud));

  return EXIT_SUCCESS;
}
