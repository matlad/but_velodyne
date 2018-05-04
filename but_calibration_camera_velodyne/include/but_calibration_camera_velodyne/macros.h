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
#include <but_calibration_camera_velodyne/color.h>

/**
 * @brief pomocné makro pro makro BUT_STR
 */
#define STR_(val) #val

/**
 * @brief Vátí hodnotu jako string (obalí val uvozovkami)
 */
#define BUT_STR(val) STR_(val)


#ifdef DEBUG
  #define DEBUG_STREAM(stream)\
     cout << COLOR_DEBUG << (stream) << COLOR_DEBUG_END << endl;
#else
  #define DEBUG_STREAM(stream)
#endif

/**
 * @brief Otevře okno a zobrazí obrázek.
 * @param image obrázek k zobrazení
 * @param char * title titulek okna
 */
#define SHOW_IMAGE(image, title)\
  cv::namedWindow((title), CV_WINDOW_AUTOSIZE);\
  cv::imshow((title), image);\
  cv::waitKey(0);\
  cv::destroyWindow((title));

/**
 * @brief Konstrukce matice o rozměrech 3x1 naplněnou nulami
 */
#define VEC_3D cv::Mat(3,1,CV_64FC1,cv::Scalar(0))

/**
 * @brief
 */
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

/**
 * @brief nastaví model Coefficients, aby odpovídaly kruhu
 */
#define CIRCLE_3D(modelCoefficients, center, radii)                              \
  (modelCoefficients).values.resize (7);                                       \
  (modelCoefficients).values[0] = (center).x;                                  \
  (modelCoefficients).values[1] = (center).y;                                  \
  (modelCoefficients).values[2] = (center).z;                                  \
  (modelCoefficients).values[3] = 0;                                           \
  (modelCoefficients).values[4] = 0;                                           \
  (modelCoefficients).values[5] = 0.001;                                       \
  (modelCoefficients).values[6] = (radii);

/**
 * @brief Inicializace proměnných pro čekací kolečko
 * k pootočení kolečka použíjte IN_PROGRES_SPIN()
 */
#define IN_PROGRES_INIT()                                                      \
  int ___time = 0;                                                             \
  char ___circle[4] = {'|','/','-','\\' };                                     \

/**
 * @brief Pootočí čekacím kolečkem.
 * Vyžaduje aby předtím byla zavolána inicializace IN_PROGRES_INIT()
 */
#define IN_PROGRES_SPIN()                                                      \
  std::cout << "\r" << ___circle[___time%4] << std::flush;                     \
  ___time++;

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_MACROS_H
