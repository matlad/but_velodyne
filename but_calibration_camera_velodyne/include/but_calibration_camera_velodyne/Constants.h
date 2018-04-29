/**
 * @file     Constants.h
 * @encoding UTF-8
 * @date     15.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_CONSTANTS_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_CONSTANTS_H
#include <opencv2/core/types.hpp>

namespace but::calibration_camera_velodyne {

const cv::Point X = {0, 0};
const cv::Point Y = {1, 0};
const cv::Point Z = {2, 0};

#define RED 2
#define GREEN 1
#define BLUE 0

typedef cv::Mat RVec;
typedef cv::Mat TVec;

}
#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CONSTANTS_H
