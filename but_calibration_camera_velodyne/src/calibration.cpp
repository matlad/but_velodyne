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

using cv::Mat;
using cv::imread;
using pcl::io::loadPCDFile;
using But::calibration_camera_velodyne::Calibrator;
using But::calibration_camera_velodyne::Velodyne::VPointCloud;
using But::calibration_camera_velodyne::CameraParameters;
using But::calibration_camera_velodyne::Calibration6DoF;
using std::cerr;
using std::endl;

int error(const char *msg) {
  cerr << msg << "\n"
	   << " <frame> <camera-parameters> <point-cloud>"
	   << " <circles_distance> <circles_radius>"
	   //<< " <x> <y> <z> <rot-x> <rot-y> <rot-z>"
	   << endl;

  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {

  if (argc != 6) {
	return error("Chybný počet argumentů");
  }

  Mat image = imread(argv[1]);
  if (image.empty()) {
	return error("Chyba při načítání obrázku");
  }

  CameraParameters cameraParameters;

  cv::FileStorage fs_P(argv[2], cv::FileStorage::READ);
  fs_P["P"] >> cameraParameters.P;
  fs_P["D"] >> cameraParameters.D;
  fs_P["K"] >> cameraParameters.K;
  fs_P.release();

  VPointCloud pointCloud;
  pcl::io::loadPCDFile(argv[3], pointCloud);
  if (pointCloud.empty()) {
	return error("Chyba při načítání pointCloud");
  }

  auto circleDistance = strtof(argv[4], nullptr);
  auto circleRadius = strtof(argv[5], nullptr);

  Calibrator calibrator(circleDistance, circleRadius);
  calibrator.setCameraParameters(cameraParameters);
  calibrator.setImage(image);
  calibrator.setPointCloud(pointCloud);

  Calibration6DoF calibration6DoF = calibrator.calibration(true);
  calibration6DoF.print();

  return EXIT_SUCCESS;
}
