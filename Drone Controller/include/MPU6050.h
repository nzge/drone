#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <tuple>

#include "constants.h"

class MPU6050{
  public:
    MPU6050();
    void init();
    void calibrate();
    std::tuple<float, float, float, float, float, float> read();
    std::tuple<float, float, float> acc_read();
    std::tuple<float, float, float> gyro_read();
  private:
    float_t Ax, Ay, Az;
    float_t Gx, Gy, Gz;
    float_t Ax_cal, Ay_cal, Az_cal;
    float_t Gx_cal, Gy_cal, Gz_cal;
};


#endif