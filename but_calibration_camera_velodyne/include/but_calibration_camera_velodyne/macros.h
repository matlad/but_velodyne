/**
 * @file     macros.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H

#include "opencv2/highgui.hpp"

/**
 * @brief pomocné makro pro makro BUT_STR
 */
#define STR_(val) #val

/**
 * @brief Vátí hodnotu jako string (obalí val uvozovkami)
 */
#define BUT_STR(val) STR_(val)

/**
 * @brief Otevře okno a zobrazí obrázek.
 * @param image oprázek k zobraszení
 * @param char * title titulek okna
 */
#define SHOW_IMAGE(image,title)\
  cv::namedWindow((title), CV_WINDOW_AUTOSIZE);\
  cv::imshow((title), image);\
  cv::waitKey(0);\
  cv::destroyWindow((title));\

#define VEC_3D cv::Mat(3,1,CV_64FC1,cv::Scalar(0))



#endif //BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H
