#include <Arduino.h>
#include <Wire.h>
#include <tuple>

#include "MPU6050.h"
#include "constants.h"

MPU6050::MPU6050() {
  Ax_cal = 0;
  Ay_cal = 0;
  Az_cal = 0;
  Gx_cal = 0;
  Gy_cal = 0;
  Gz_cal = 0;
}

void MPU6050::init() { 
  Wire.beginTransmission(0x68); // MPU6050 I2C address
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission();

  // Set Digital Low Pass Filter (DLPF)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // DLPF config register
  Wire.write(0x05); // 10 Hz cutoff frequency
  Wire.endTransmission();

  // Configure Gyro (±500°/s range)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Gyro config register
  Wire.write(0x08); // Set range (65.5 LSB/deg/s)
  Wire.endTransmission();

  // Configure Accelerometer (±2g range)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accel config register
  Wire.write(0x00); // Set range (16384 LSB/g)
  Wire.endTransmission();
  
  calibrate();
}

//calibration sequence  
void MPU6050::calibrate() {
  float_t sumAx = 0, sumAy = 0, sumAz = 0;
  float_t sumGx = 0, sumGy = 0, sumGz = 0;
  // Collect accelerometer and gyro data to compute offsets
  for (int i = 0; i < calibrationSamples; i++) {
    auto [ax, ay, az, gx, gy, gz] = read();
    
    Serial.print("Acc: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.println(az);

    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz); 
 
    // Sum the errors for each axis
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    
    Serial.print("Sum of Acc: ");
    Serial.print(sumAx); Serial.print(", ");
    Serial.print(sumAy); Serial.print(", ");
    Serial.println(sumAz);

    Serial.print("Sum of Gyro: ");
    Serial.print(sumGx); Serial.print(", ");
    Serial.print(sumGy); Serial.print(", ");
    Serial.println(sumGz);

    delay(10); // Small delay between readings
  }

  // Calculate average errors
  Ax_cal = sumAx / calibrationSamples;
  Ay_cal = sumAy / calibrationSamples;
  Az_cal = (sumAz / calibrationSamples) - 1; // Subtract 1g from Z axis
  Gx_cal = sumGx / calibrationSamples;
  Gy_cal = sumGy / calibrationSamples;
  Gz_cal = sumGz / calibrationSamples;

  // Print calibration results
  Serial.println("Calibration done");
  Serial.print("Acc Errors: ");
  Serial.print(Ax_cal);
  Serial.print(" ");
  Serial.print(Ay_cal);
  Serial.print(" ");
  Serial.println(Az_cal);

  Serial.print("Gyro Errors: ");
  Serial.print(Gx_cal);
  Serial.print(" ");
  Serial.print(Gy_cal);
  Serial.print(" ");
  Serial.println(Gz_cal);
}

std::tuple<float, float, float, float, float, float>  MPU6050::read(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  
  Wire.requestFrom(0x68, 14); // Request 14 bytes
  int16_t rawAx=Wire.read()<<8 | Wire.read();
  int16_t rawAy=Wire.read()<<8 | Wire.read();
  int16_t rawAz=Wire.read()<<8 | Wire.read();
  int16_t temp=Wire.read()<<8 | Wire.read();
  int16_t rawGx=Wire.read()<<8 | Wire.read();
  int16_t rawGy=Wire.read()<<8 | Wire.read();
  int16_t rawGz=Wire.read()<<8 | Wire.read();

  float_t Ax = (float)(rawAx) / LSB_acc - Ax_cal; // convert to G
  float_t Ay = (float)(rawAy) / LSB_acc - Ay_cal;
  float_t Az = (float)(rawAz) / LSB_acc - Az_cal;
  float_t Gx = (float)(rawGx) / LSB_gyr - Gx_cal;
  float_t Gy = (float)(rawGy) / LSB_gyr - Gy_cal;
  float_t Gz = (float)(rawGz) / LSB_gyr - Gz_cal;

  return std::make_tuple(Ax, Ay, Az, Gx, Gy, Gz);
}

std::tuple<float, float, float>  MPU6050::gyro_read(){
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Starting register for gyro readings
  Wire.endTransmission(false); 

  Wire.requestFrom(0x68,6);// Request 6 bytes
  int16_t rawGx=Wire.read()<<8 | Wire.read();
  int16_t rawGy=Wire.read()<<8 | Wire.read();
  int16_t rawGz=Wire.read()<<8 | Wire.read();
  
  float Gx = (float)(rawGx) / LSB_gyr - Gx_cal;
  float Gy = (float)(rawGy) / LSB_gyr - Gy_cal;
  float Gz = (float)(rawGz) / LSB_gyr - Gz_cal;
  return std::make_tuple(Gx, Gy, Gz);
}


std::tuple<float, float, float>  MPU6050::acc_read(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6); // Request 6 bytes
  int16_t rawAx=Wire.read()<<8 | Wire.read();
  int16_t rawAy=Wire.read()<<8 | Wire.read();
  int16_t rawAz=Wire.read()<<8 | Wire.read();

  float Ax = (float)(rawAx) / LSB_acc - Ax_cal; // convert to G
  float Ay = (float)(rawAy) / LSB_acc - Ay_cal;
  float Az = (float)(rawAz) / LSB_acc - Az_cal;
  return std::make_tuple(Ax, Ay, Az);
}

