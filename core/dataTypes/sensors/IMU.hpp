//
// Created by root on 5/15/21.
//

#ifndef NAV24_IMU_H
#define NAV24_IMU_H

#include <string>
#include <vector>
#include <memory>

#include "SharedQueue.hpp"


namespace NAV24 {
    struct IMU_Data {

        IMU_Data() : ts(0.0), gyro{}, accel{} {}
        IMU_Data(double ts, float gx, float gy, float gz, float ax, float ay, float az) :
                ts(ts), gyro{gx, gy, gz}, accel{ax, ay, az}
        {}

        IMU_Data(double ts, const float g[3], const float a[3]) : ts(ts), gyro{}, accel{} {

            for (unsigned char i = 0; i < 3; i++) {
                gyro[i] = g[i];
                accel[i] = a[i];
            }
        }

        double ts;
        float accel[3]; //ax, ay, az
        float gyro[3];  //gx, gy, gz
    };

    typedef std::shared_ptr<IMU_Data> IMU_DataPtr;

    typedef SharedQueue<IMU_DataPtr> IMU_Queue;
    typedef std::shared_ptr<IMU_Queue> IMU_QueuePtr;

} // NAV24


#endif //NAV24_IMU_H
