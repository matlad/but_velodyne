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

namespace But::calibration_camera_velodyne {
class Calibration6DoF {
 public:
  /**
   * @brief vektor six degras of freadom
   * transtation x y z
   * a rotation x y z v
   * tomto pořadí
   */
  std::vector<float> DoF;

  /**
   * @brief Skoré jak moc je tato kalibrace přesná
   */
  float value; // NaN = wrong calibration

 public:
  Calibration6DoF(float x, float y, float z,
                  float x_r, float y_r, float z_r,
                  float val
  ) {
    set(x, y, z, x_r, y_r, z_r, val);
  }

  /**
   * @param tvec
   * @param rvec
   * @param val
   * @see ::set(cv::Mat tvec, cv::Mat rvec, float val)
   */
  Calibration6DoF(cv::Mat tvec, cv::Mat rvec, float val)
  {
    set(tvec,rvec,val);
  };

  Calibration6DoF(){
    value = 0;
    DoF.resize(6, 0);
  }

  static Calibration6DoF wrong() {
    return Calibration6DoF(0, 0, 0, 0, 0, 0, NAN);
  }

  bool isGood() {
    return !std::isnan(value);
  }

  void set(float x, float y, float z, float x_r, float y_r, float z_r, float val)
  {
    value = val;

    DoF.clear();
    DoF.push_back(x);
    DoF.push_back(y);
    DoF.push_back(z);
    DoF.push_back(x_r);
    DoF.push_back(y_r);
    DoF.push_back(z_r);
  }

  /**
   * @brief nastaví 6DoF
   * @param tvec matice 1x3 pro transformaci x,y,z
   * @param rvec matice 1x3 pro rotaci x,y,z
   * @param val score
   */
  void set(cv::Mat tvec, cv::Mat rvec, float val);

  float tX();
  float tY();
  float tZ();
  float rX();
  float rY();
  float rZ();

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
