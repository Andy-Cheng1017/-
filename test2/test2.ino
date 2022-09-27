#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t  gyroRate;
int16_t  gyroX[120];
int32_t  sum;
float gyroAngle = 0;
unsigned long currTime, prevTime = 0, loopTime;

void setup() {
  mpu.initialize();
  Serial.begin(115200);
}

void loop() {
  currTime = micros();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  sum = 0;
  for (int i = 0 ; i < 120; i++ )
  {
    gyroX[i] = mpu.getRotationY();
    sum = sum + gyroX[i];
  }
  sum = (sum / 120) + 125;
  //  gyroX = mpu.getRotationX();
  gyroRate = map(sum, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate * loopTime / 1000000;

  Serial.println(gyroAngle);
}
