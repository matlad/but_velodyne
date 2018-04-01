/**
 * @file     Calibration6DoF.h
 * @encoding UTF-8
 * @date     1.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATION6DOF_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATION6DOF_H

#include <cstdlib>
#include <cstdio>
#include <but_calibration_camera_velodyne/Similarity.h>
//#include "opencv2/opencv.hpp"

namespace But::calibration_camera_velodyne {
class Calibration6DoF {
 public:
  std::vector<float> DoF;

  float value; // NaN = wrong calibration

 public:
  Calibration6DoF(float x,
                  float y,
                  float z,
                  float x_r,
                  float y_r,
                  float z_r,
                  float val) {
    set(x, y, z, x_r, y_r, z_r, val);
  }

  Calibration6DoF() {
    value = 0;
    DoF.resize(6, 0);
  }

  static Calibration6DoF wrong() {
    return Calibration6DoF(0, 0, 0, 0, 0, 0, NAN);
  }

  bool isGood() {
    return !std::isnan(value);
  }

  void set(float x,
           float y,
           float z,
           float x_r,
           float y_r,
           float z_r,
           float val) {
    value = val;

    DoF.clear();
    DoF.push_back(x);
    DoF.push_back(y);
    DoF.push_back(z);
    DoF.push_back(x_r);
    DoF.push_back(y_r);
    DoF.push_back(z_r);
  }

  void set() {

  }

  bool operator<=(Calibration6DoF &other) {
    return this->value <= other.value;
  }

  void operator+=(Calibration6DoF &other) {
    this->value += other.value;
    ROS_ASSERT(this->DoF.size() == other.DoF.size());
    for (size_t i = 0; i < DoF.size(); i++) {
      this->DoF[i] += other.DoF[i];
    }
  }

  void operator/=(float div) {
    this->value /= div;
    for (size_t i = 0; i < DoF.size(); i++) {
      this->DoF[i] /= div;
    }
  }

  void print(void) {
    cout << "6DoF: [" << DoF[0] << ", " << DoF[1] << ", " << DoF[2] << ", "
         << DoF[3] << ", "
         << DoF[4] << ", " << DoF[5] << "]" << " #score: " << value << ";\t"
         << endl;
  }
};
}
#endif //BUT_CALIBRATION_CAMERA_VELODYNE_CALIBRATION6DOF_H
