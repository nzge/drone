#include <Arduino.h>
#include <Wire.h>

#include <constants.h>
#include <MPU6050.h>

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
  auto [gx, gy, gz] = mpu.gyro_read();
  auto [ax, ay, az] = mpu.acc_read();

  Serial.print("Gx: "); Serial.print(gx);
  Serial.print(" Gy: "); Serial.print(gy);
  Serial.print(" Gz: "); Serial.println(gz);

  Serial.print(" | Ax: "); Serial.print(ax);
  Serial.print(" Ay: "); Serial.print(ay);
  Serial.print(" Az: "); Serial.println(az);

  delay(1000);
}