#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// Library being read from
MPU6050 mpu;
Servo rollServo;
Servo pitchServo;
int rollservo_pin = 41;
int pitchservo_pin = 21;

float rollServo_max = 15;
float rollServo_min = 165;
float pitchServo_max = 60;
float pitchServo_min = 120;

const double r2d = 180 / M_PI;

uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
Quaternion q;
float euler[3];

// MPU control/status vars
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Setup script to run prechecks before being read as an output
void setup() {
  Serial.begin(115200);
  while(!Serial){}
  rollServo.attach(rollservo_pin);
  pitchServo.attach(pitchservo_pin);
  Serial.print("Centering the servos...");
  pitchServo.write(90);
  rollServo.write(90);

  Serial.println("Enter an angle.");

  // setting up I2C connection to IMU 
  Serial.println("-----------------------");
  Serial.println("Setting up I2C connection...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize IMU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  // make sure it worked (returns 0 if so)
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
  } 
    
  // verify connection with IMU
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  while(!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    get_IMU_state();
  }
}

char rx_byte = 0;
String rx_str = "";
boolean not_number = false;
int result;

// Depending on what is needed to be actuated.
// Set 0 for pitch and 1 for roll;
int tester = 1;

// This reads the get_IMU_state function below in real time and controls the servo.
void loop() {  
  // Servo Control
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();
    
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      rx_str += rx_byte;
    }
    else if (rx_byte == '\n') {
      // end of string
      if (not_number) {
        Serial.println("Not a number");
      }
      else {
        result = rx_str.toInt();
        // print the result
        Serial.print(result);
        Serial.println("Enter an angle.");
      }
      not_number = false;         // reset flag
      rx_str = "";          // clear the string for reuse
    }
    else {
      // non-number character received
      not_number = true;    // flag a non-number
    }
    Serial.println(result);
  if (tester == 1)    {
    if (result > rollServo_min || result < rollServo_max){
      Serial.print("Error, please enter an angle within bounds");
      result = 90;
    }
  rollServo.write(result);
  }
  else if (tester == 0);{
    if (result > pitchServo_min || result < pitchServo_max){
      Serial.print("Error, please enter an angle within bounds");
      result = 90;
    }
  pitchServo.write(180 - result);
  }
  } //end: if (Serial.available() > 0) 

  // IMU readings in real time
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    get_IMU_state();
  }
}

// Outputs to user to read data specified above
void get_IMU_state(){
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);

// Print out the values
  Serial.print("Rotation X: ");
  Serial.println(r2d * euler[2]);
  Serial.print("Rotation Y: ");
  Serial.println(r2d * euler[1]);
  Serial.print("Rotation Z: ");
  Serial.println(r2d * euler[0]);

  Serial.println("---------------------");
}
