#ifndef CONSTANTS_H
#define CONSTANTS_H

// IMU
const int calibrationSamples = 1000; // Number of calibration samples to average

inline float LSB_gyr = 65.5f; // LSB/deg/s sensitivity
inline float LSB_acc = 16384.0f; // LSB/g sensitivity
inline float sampleFreq = 50.0f; // IMU sample frequency
inline float sampleTime = (1.0f/sampleFreq) * 1000; // IMU sample time
//


#endif