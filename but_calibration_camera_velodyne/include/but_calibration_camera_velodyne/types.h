/**
 * @file     types.h
 * @encoding UTF-8
 * @date     29.4.18
 * @author   Adam Mátl <xmatla00@stud.fit.vutbr.cz> <matla@matla.cz>
 */
#ifndef BUT_CALIBRATION_CAMERA_VELODYNE_TYPES_H
#define BUT_CALIBRATION_CAMERA_VELODYNE_TYPES_H

namespace but::calibration_camera_velodyne {

/**
 * @brief literal pro zapsání barvy jednoho z kanálu rgb tedy hodnoty 0-255
 * @return
 */
unsigned char operator "" _rgb_c(unsigned long long in);


}

#endif //BUT_CALIBRATION_CAMERA_VELODYNE_TYPES_H
