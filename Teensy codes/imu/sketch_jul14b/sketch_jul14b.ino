#include <MPU6050_6Axis_MotionApps20.h>
#include "ubx.h"
#include <SPI.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// Library being read from
MPU6050 mpu;

const double r2d = 180 / M_PI;

 uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
 Quaternion q;
 VectorInt16 aa;
 VectorFloat gravity;
 float ypr[3];
 float euler[3];

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// Setup script to run prechecks before being read as an output
void setup() {
  Serial.begin(115200);

  // setting up I2C connection to IMU 
  Serial.println("-----------------------");
  Serial.println("Setting up I2C connection...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize IMU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
    
  // verify connection with IMU
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
}

// Outputs to user to read data specified above
void get_IMU_state(){
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetEuler(euler, &q);
  mpu.dmpGetAccel(&aa, fifoBuffer);

//  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(aa[1]);
//
//  Serial.print("Rotation X: ");
//  Serial.print(Euler[1]);

  Serial.println("");
  delay(500);
}
