#include <MPU6050.h>
#include <constants.h>
#include <Wire.h>

void MPU6050::init(void) { 
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

  // Configure Accelerometer (±4g range)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accel config register
  Wire.write(0x08); // Set range (8192 LSB/g)
  Wire.endTransmission();

  //calibrate();
}

void MPU6050::calibrate() {
  //calibration sequence
  const int numSamples = 1000; // Number of samples to average

   // Collect accelerometer and gyro data to compute offsets
   for (int i = 0; i < numSamples; i++) {
    auto [ax, ay, az] = acc_read();  // Replace with your accelerometer read function
    auto [gx, gy, gz] = gyro_read();     // Replace with your gyroscope read function

    // Sum the errors for each axis
    AccErrorX += ax;
    AccErrorY += ay;
    AccErrorZ += az;

    GyroErrorX += gx;
    GyroErrorY += gy;
    GyroErrorZ += gz;
    
    delay(1); // Small delay between readings
  }

  // Calculate average errors
  AccErrorX /= numSamples;
  AccErrorY /= numSamples;
  AccErrorZ /= numSamples;
  GyroErrorX /= numSamples;
  GyroErrorY /= numSamples;
  GyroErrorZ /= numSamples;

  // Print calibration results
  Serial.print("Calibration done");
  Serial.print("Acc Errors: ");
  Serial.print(AccErrorX);
  Serial.print(" ");
  Serial.print(AccErrorY);
  Serial.print(" ");
  Serial.print(AccErrorZ);

  Serial.print("Gyro Errors: ");
  Serial.print(GyroErrorX);
  Serial.print(" ");
  Serial.print(GyroErrorY);
  Serial.print(" ");
  Serial.print(GyroErrorZ);
}


std::tuple<float, float, float>  MPU6050::gyro_read(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Starting register for gyro readings
  Wire.endTransmission(false); 

  Wire.requestFrom(0x68,6);// Request 6 bytes
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  Gx=(float)GyroX/LSB_sense;
  Gy=(float)GyroY/LSB_sense;
  Gz=(float)GyroZ/LSB_sense;

  // Subtract the gyro calibration errors
  Gx = Gx - GyroErrorX;
  Gy = Gy - GyroErrorY;
  Gz = Gz - GyroErrorZ;
  return std::make_tuple(Gx, Gy, Gz);
}


std::tuple<float, float, float>  MPU6050::acc_read(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6); // Request 6 bytes
  int16_t rawX=Wire.read()<<8 | Wire.read();
  int16_t rawY=Wire.read()<<8 | Wire.read();
  int16_t rawZ=Wire.read()<<8 | Wire.read();
  Ax = (float)rawX / 8192.0; // Convert to g (assuming ±4g range)
  Ay = (float)rawY / 8192.0;
  Az = (float)rawZ / 8192.0;
  
  // Subtract the accelerometer calibration errors
  Ax = Ax - AccErrorX;
  Ay = Ay - AccErrorY;
  Az = Az - AccErrorZ;
  return std::make_tuple(Ax, Ay, Az);
}

