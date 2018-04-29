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
#include <but_calibration_camera_velodyne/Calibrator.h>
#include <but_calibration_camera_velodyne/Velodyne.h>
#include <but_calibration_camera_velodyne/FishEyeCamera.h>

#define ARG_FRAME 1
#define ARG_CAMERA_PARAMETERS 2
#define ARG_POINT_CLOUD 3
#define ARG_CIRCLES_DISTANCE 4
#define ARG_CIRCLES_RADIUS 5

using cv::Mat;
using cv::imread;
using pcl::io::loadPCDFile;
using but::calibration_camera_velodyne::Calibrator;
using but::calibration_camera_velodyne::Velodyne::VPointCloud;
using but::calibration_camera_velodyne::FishEyeCamera;
using but::calibration_camera_velodyne::Calibration6DoF;
using std::cerr;
using std::endl;

int error(const char *msg) {
  cerr << msg << "\n očekávané parametry:"
       << " <frame> <camera-parameters> <point-cloud> <circles_distance> <circles_radius>\n\n"
       << "  <frame>             - Soubor se snímkem z kamery"
       << "  <camera-parameters> - Soubor s parametry kamery"
       << "  <point-cloud>       - Soubor s mračnem bodů"
       << "  <circles_distance>  - Vzdálenost mezi středy kruhů"
       << "  <circles_radius>    - Radius kruhů"
       << endl;

  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {

  if (argc != 6) {
	return error("Chybný počet argumentů");
  }

  Mat image = imread(argv[ARG_FRAME]);
  if (image.empty()) {
	return error("Chyba při načítání obrázku");
  }

  FishEyeCamera camera;

  cv::FileStorage fs_P(argv[ARG_CAMERA_PARAMETERS], cv::FileStorage::READ);
  fs_P["P"] >> camera.P;
  fs_P["D"] >> camera.D;
  fs_P["K"] >> camera.K;
  fs_P.release();


  camera.tvec = VEC_3D;
  camera.rvec = VEC_3D;

  VPointCloud pointCloud;
  pcl::io::loadPCDFile(argv[ARG_POINT_CLOUD], pointCloud);
  if (pointCloud.empty()) {
	return error("Chyba při načítání pointCloud");
  }

  auto circleDistance = strtod(argv[ARG_CIRCLES_DISTANCE], nullptr);
  auto circleRadius = strtod(argv[ARG_CIRCLES_RADIUS], nullptr);

  Calibrator calibrator(circleDistance, circleRadius);
  calibrator.setCamera(&camera);
  calibrator.setImage(image);
  calibrator.setPointCloud(pointCloud);

  Calibration6DoF calibration6DoF = calibrator.calibration(true);
  calibration6DoF.print();

  return EXIT_SUCCESS;
}
