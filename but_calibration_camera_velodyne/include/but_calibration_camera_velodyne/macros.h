/**
 * @file     macros.h
 * @encoding UTF-8
 * @date     31.3.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H

#include <sys/ioctl.h>
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
#define SHOW_IMAGE(image, title)\
  cv::namedWindow((title), CV_WINDOW_AUTOSIZE);\
  cv::imshow((title), image);\
  cv::waitKey(0);\
  cv::destroyWindow((title));

#define VEC_3D cv::Mat(3,1,CV_64FC1,cv::Scalar(0))

#define ASSERT_IS_VEC_3D(vec)\
assert((vec).cols == 1);\
assert((vec).rows == 3);\
assert((vec).type() == CV_64FC1);

#define GENERATE_3D_POINTS()\
std::vector<cv::Point3d> points;                                               \
                                                                               \
double x, y, z;                                                                \
                                                                               \
x = 0; y = 0; z = +.5;                                                         \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = .5; y = .5; z = -.5;                                                       \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = .5; y = .5; z = .5;                                                        \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = -.5; y = .5; z = .5;                                                       \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = -.5; y = .5; z = -.5;                                                      \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = .5; y = -.5; z = -.5;                                                      \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = -.5; y = -.5; z = -.5;                                                     \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = -.5; y = -.5; z = .5;                                                      \
points.push_back(cv::Point3d(x, y, z));                                        \
                                                                               \
x = -0.336698; y = -0.0989147; z = 1.37389;                                    \
points.push_back(cv::Point3d(x, y, z));

#define CIRCLE_3D(modelCoefficients, center, radii)                              \
  (modelCoefficients).values.resize (7);                                       \
  (modelCoefficients).values[0] = (center).x;                                  \
  (modelCoefficients).values[1] = (center).y;                                  \
  (modelCoefficients).values[2] = (center).z;                                  \
  (modelCoefficients).values[3] = 0;                                           \
  (modelCoefficients).values[4] = 0;                                           \
  (modelCoefficients).values[5] = 0.001;                                       \
  (modelCoefficients).values[6] = (radii);

#define IN_PROGRES_INIT()                                                      \
  struct winsize ___window;                                                    \
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &___window);                                \
  int ___time = 0;                                                             \
  int ___before = 0;                                                           \
  int ___after = 0;                                                            \
  int ___slideWihgt = ___window.ws_col == 2 ? 1 : ___window.ws_col -2;

#define IN_PROGRES_SPIN()                                                      \
  ___before = (___time % (___slideWihgt)) +1;                                  \
  ___after  = (___window.ws_col) - 1 -  ___before;                             \
  std::cout << "\r" << "[" << std::setw(___before) << "#"                      \
            << std::setw(___after) << "]" << std::flush;                       \
  ___time++;

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H
