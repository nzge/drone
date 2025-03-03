#include <Arduino.h>
#include <Wire.h>
#include <tuple>

#include <constants.h>
#include "MPU6050.h"

#include "madgwick.h"
#include "pid.h"
#include "motor.h"

#include "MadgwickAHRS.h"

//objects
MPU6050 mpu;
Madgwick filter;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

void setup() {
  Serial.begin(57600); //initialize serial communication: 57600 baud rate
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Wire.setClock(400000);
  Wire.begin();

  filter.begin(sampleFreq); // 50 Hz sample rate

  Serial.println("Starting MPU6050");
  mpu.init();

  delay(250);
}

void loop() {
  auto [ax, ay, az, gx, gy, gz] = mpu.read();

  filter.updateIMU(gx, gy, gz, ax, ay, az);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  
  Serial.print(">Roll:"); 
  Serial.println(roll);

  Serial.print(">Pitch: "); 
  Serial.println(pitch);

  Serial.print(">Yaw: "); 
  Serial.println(yaw); 

  Serial.print(">Ax: "); 
  Serial.println(ax, 6); 
  Serial.print(">Ay: "); 
  Serial.println(ay, 6); 
  Serial.print(">Az: "); 
  Serial.println(az, 6); 
  Serial.print(">Gx: "); 
  Serial.println(gx, 6); 
  Serial.print(">Gy: "); 
  Serial.println(gy, 6); 
  Serial.print(">Gz: "); 
  Serial.println(gz,6);

  /*
  // Send data as a single CSV-formatted line
  Serial.print(ax, 6); Serial.print(",");
  Serial.print(ay, 6); Serial.print(",");
  Serial.print(az, 6); Serial.print(",");
  Serial.print(gx, 6); Serial.print(",");
  Serial.print(gy, 6); Serial.print(",");
  Serial.println(gz);  // Newline at the end to indicate one complete reading  */


  /*
  Serial.print(">Roll:"); Serial.print(roll); Serial.print(",");
  Serial.print(">Pitch: "); Serial.print(pitch); Serial.print(",");
  Serial.print(">Yaw: "); Serial.println(yaw); */

  delay(sampleTime);
}
