/**
 * @file     types.cpp
 * @encoding UTF-8
 * @date     29.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#include <but_calibration_camera_velodyne/types.h>
#include <assert.h>

unsigned char but::calibration_camera_velodyne::operator "" _rgb_c(unsigned long long in) {
  assert(in < 256);
  return static_cast<unsigned char>(in);
}
