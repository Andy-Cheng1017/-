#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accY, accZ;
float accAngle;

void setup() {
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {
  //  accZ = mpu.getAccelerationZ();
  //  accX = mpu.getAccelerationX();
  //
  Serial.print(mpu.getAccelerationX() >> 8);
  Serial.print("\t");
  Serial.print(mpu.getAccelerationY() >> 8);
  Serial.print("\t");
  Serial.print(mpu.getAccelerationZ() >> 8);
  Serial.print("-----\t");
  Serial.print(mpu.getRotationX() >> 8);
  Serial.print("\t");
  Serial.print(mpu.getRotationY() >> 8);
  Serial.print("\t");
  Serial.println(mpu.getRotationZ() >> 8);
  delay(100);
  //  accAngle = atan2(accY, accZ)*RAD_TO_DEG;

  //  if(isnan(accAngle));
  //  else
  //    Serial.println(accAngle);
}
