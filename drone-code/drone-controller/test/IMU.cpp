#include <Arduino.h>
#include <Wire.h>
#include <tuple>

#include "constants.h"
#include "MPU6050.h"
#include "MPU6050.cpp"

//objects
MPU6050 mpu;

// put function declarations here:


void setup() {
  Serial.begin(57600); //initialize serial communication: 57600 baud rate
  Serial.println("MPU6050 Test");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  mpu.init();

  delay(250);
}


void loop() {
  auto [ax, ay, az, gx, gy, gz] = mpu.read();

  // Send data as a single CSV-formatted line
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);  // Newline at the end to indicate one complete reading

  delay(200);
}

/*
void loop(){
  auto [ax, ay, az, gx, gy, gz] = mpu.read();
  // Use stringstream to format the IMU data

  std::stringstream ss;
  ss << ax << "," << ay << "," << az << "," << gx << "," << gy << "," << gz;

  // Print the formatted data to Serial (OpenGL will read this)
  Serial.println(ss.str());

  delay(200);
} */

/*
void loop() {
  auto [gx, gy, gz] = mpu.gyro_read();
  auto [ax, ay, az] = mpu.acc_read();

  Serial.print("Gx: "); Serial.print(gx);
  Serial.print(" Gy: "); Serial.print(gy);
  Serial.print(" Gz: "); Serial.print(gz);

  Serial.print(" | Ax: "); Serial.print(ax);
  Serial.print(" Ay: "); Serial.print(ay);
  Serial.print(" Az: "); Serial.println(az);

  delay(200);
}*/

