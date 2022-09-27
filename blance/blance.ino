#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "RF24.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LeftMotorDirection 1 //the spin of the left motor(top view)，1 is clockwise, 0 is counterclockwise
#define RightMotorDirection 0 //the spin of the right motor(top view)，1 is clockwise, 0 is counterclockwise
#define INA 5
#define INB 6
#define INC 9
#define IND 10
int Kp = 90;
int Ki = 300;
int Kd = 2;
#define target -50
//-----------------------------------------------------------------------
double Setpoint, Input, Output;
PID PIDCTRL(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
byte Reverse, Forward;

//-----------------------------------------------------------------------
MPU6050 mpu;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//---------------------------------------------------------------------------------------------- -
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//---------------------------------------------------------------------------------------------- -
RF24 rf24(7, 8);                                  //(ce,cs)
const byte pipe = 1;                              // 指定通道編號
const byte addr[] = "1Node";                      //Address
//-----------------------------------------------------------------------------
int msg[4];                  //主要接收訊息變數 4byte 32bite
//所有馬達狀態

unsigned long CurrentTime ;          //設定起始時間ms
unsigned long TimeOut;               //最終時間ms

//float CurrentAngle;

void setup() {
  //  delay(5000);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //  Serial.begin(115200);
  //  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(37);
  mpu.setYGyroOffset(51);
  mpu.setZGyroOffset(-12);
  mpu.setXAccelOffset(-1418);
  mpu.setYAccelOffset(219);
  mpu.setZAccelOffset(1000);
  mpu.setDMPEnabled(true);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //    Serial.print(F("DMP Initialization failed (code "));
    //    Serial.print(devStatus);
    //    Serial.println(F(")"));
  }
  //-----------------------------------------------------------------------------------------
  Setpoint = target;
  PIDCTRL.SetMode(AUTOMATIC);
  PIDCTRL.SetSampleTime(1);
  PIDCTRL.SetOutputLimits(-255, 255);
  //-----------------------------------------------------------------------------------------
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);
  rf24.begin();
  rf24.setChannel(83);               // 設定頻道編號
  rf24.openWritingPipe(addr);  // 開啟通道和位址
  rf24.openReadingPipe(1, addr);
  rf24.startListening();               // 開始監聽無線廣播
  rf24.setPALevel(RF24_PA_MAX);      //設最高功率接收
  rf24.setDataRate(RF24_2MBPS) ;
  //  rf24.setPayloadSize(16);
  //-----------------------------------------------------------------------------------------

  //  while (millis() - CurrentTime <= 10000) {
  //    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
  //      mpu.dmpGetQuaternion(&q, fifoBuffer);
  //      mpu.dmpGetGravity(&gravity, &q);
  //      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //      //      Serial.print("ypr\t");
  //      //      Serial.print(ypr[0] * 180 / M_PI);
  //      //      Serial.print("\t");
  //      //      Serial.print(ypr[1] * 180 / M_PI);
  //      //      Serial.print("\t");
  //      //      Serial.print(ypr[2] * 180 / M_PI);
  //      //      Serial.println();
  //    }
  //  }

}

void loop() {
  rf24.read(&msg , sizeof(msg));
  if (rf24.available(&pipe)) {
    rf24.read(&msg , sizeof(msg));
    rf24.stopListening();
    rf24.write(&msg, sizeof(msg));
    rf24.startListening();
    PIDCTRL.SetTunings(msg[0], msg[1], msg[2]);
    PIDCTRL.SetPoint(msg[3]);
  }
  PIDCTRL.Compute();
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Input = ypr[1] * 180 / M_PI;
  //  Serial.print(Input);
  //  Serial.print("\t");
  //  Serial.println(Output);
  if (Output >= 1) {
    Reverse = abs(Output);
    analogWrite(INA, Reverse * RightMotorDirection);
    analogWrite(INB, Reverse * !RightMotorDirection);
    analogWrite(INC, Reverse * LeftMotorDirection);
    analogWrite(IND, Reverse * !LeftMotorDirection);
  }
  else if (Output < 1) {
    Forward = abs(Output);
    analogWrite(INA, Forward * !RightMotorDirection);
    analogWrite(INB, Forward * RightMotorDirection);
    analogWrite(INC, Forward * !LeftMotorDirection);
    analogWrite(IND, Forward * LeftMotorDirection);
  }
}
