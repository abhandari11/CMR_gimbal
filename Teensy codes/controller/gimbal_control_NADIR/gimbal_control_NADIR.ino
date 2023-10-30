//
// Line 137: Void Setup
// Line 267: Void Loop
//
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "ubx.h"
#include <SPI.h>
#include <SD.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//*******Tunable Parameters*******//

// set this to true if you need to see serial monitor's output. Needs computer plugged in. 
// Set this to false for normal operation
bool _DEBUG = true;
// Transformation complete, goes from stow state to operation in less gitters. 
bool trans_comp = false; 

// Controller Gains
float Kp = 2.9;
float Ki = 0.0;

// Check with the servo test code. Manually test what servo.write() cmds correspond to the max and min angles. 
float _servo_center_angle = 90;
float servo_roll_max = 9;
float servo_roll_min = 168;
float servo_pitch_max = 60;
float servo_pitch_min = 120;
float stow_angle = 15;

// Range of servo movements 
// Manually check the range of mounted antenna at the max and min angles. Use a phone level.  
float max_roll_angle = 73;
float min_roll_angle = -73;
float max_pitch_angle = 30;
float min_pitch_angle = -30;

// delay between timesteps for servo to goto a new positions
float servo_delay = 10;

float max_angular_rate = 500;

// gnss baud rate 
int _gnss_baud = 115200;

// update rate: 10 millisecond
const int16_t LOOP_PERIOD_MS = 10;

// rate to check closest survey point
const int16_t HEIGHT_CHECK_PERIOD_MS = 10000;

// Number of times the relay signal needs to be consistent. 
int _relay_threshold = 10;

// Rate for SD card upload
uint8_t SD_CARD_PERIOD_MS = 200;


//**************//

const double r2d = 180 / M_PI;
const double d2r = 1 / r2d;

// Creating instances of hardware
Servo roll_servo;
Servo pitch_servo;
MPU6050 mpu; 
bfs::Ubx gnss(&Serial2);

// defining pins used in Teensy
const int pin_roll_servo = 41;
int pitch_servo_pin = 21;
const int relay_pin = 25;

// a struct to define position Geo-position data relative to the first point (First Common midpoint)
struct PositionGeo {
  double latitude;
  double longitude;
  float altitude;
  float distance_from_origin; 
  float deviation_from_line;
  };

bool _relay_state = false; 
int _state_counter = 0;
bool _servo_centered = false; 
bool new_file_created = false;
String file_prefix = "LOG";
String file_name = "";

// last time data was written in sd card
unsigned long last_sd_write_time;

// orientation/motion vars based on IMU library (MPU6050)
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];
float roll_angle;
float pitch_angle;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// other global variables
float target_roll_angle = 0.0;
float target_pitch_angle = 0.0;
float closest_rel_height = 10.0;
PositionGeo current_pos; 
struct PositionGeo gnd_points[2];
float line_bearing;
float line_length;
float last_servo_cmd_roll;
float last_servo_cmd_pitch;
float cum_error_roll;
float cum_error_pitch;
float control_roll_out;
float control_pitch_out;

// time related global variables
unsigned long last_height_checked_time;
unsigned long current_time, previous_time, step_prev_time;

//**********************************************************************************//
void setup() {
  step_prev_time = millis();
  // initialize Serial connection
  Serial.begin(115200);
  if (_DEBUG){
    while(!Serial) {}
  }
  gnss.Begin(115200);

  // setting the digital relay switch from Ardupilot Relay
  pinMode(relay_pin, INPUT);
  Serial.println("-----------------------");
  Serial.print("The relay status is : ");
  Serial.print(digitalRead(relay_pin));

  // attaching and setting the servos to center angle
  roll_servo.attach(pin_roll_servo);
  pitch_servo.attach(pitch_servo_pin);
  Serial.println("Centering the servos...");
  Serial.println("-----------------------");
  roll_servo.write(_servo_center_angle);
  pitch_servo.write(_servo_center_angle);
  _servo_centered = true;
  last_servo_cmd_roll = _servo_center_angle;
  last_servo_cmd_pitch = _servo_center_angle;

  // setting up I2C connection to IMU 
  Serial.println("-----------------------");
  Serial.println("Setting up I2C connection...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // For Debug mode, wait for user confirmatoin:: Needs keyboard entry
  if (_DEBUG) {
    Serial.println(F("\nSend any character to begin the loop: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  } 
  else {
    // For actual run case, delay for 2 sec. 
    delay(2000);
  }
  
  // initialize IMU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection with IMU
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // Provided offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1780);
  Serial.println();
  Serial.print("Offsets before calibration...\n");
  mpu.PrintActiveOffsets();  

  // make sure it worked (returns 0 if so)
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

  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  } 
  // Initialize SD Card
  Serial.println("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while(1){}
    }
  Serial.println("SD card initialized");

  uint8_t file_counter = 0;
  file_name = file_prefix + String(file_counter) + ".txt";
  
  while (SD.exists(file_name.c_str())){
    file_counter++;
    file_name = file_prefix + String(file_counter) + ".txt";
  }
  
  new_file_created = true;
  Serial.println(file_name);
  // wait for ready after initializing IMU:: Needs keyboard entry
  if (_DEBUG) {
    Serial.println(F("\nSend any character to begin the loop: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  }
  else {
    // For real case, wait for 2 sec. 
    delay(2000);
  }

  // Load the predefined survey points-of-interest and do necessary calculations. 
  Serial.println("Setting starting survey point as origin");
  load_survey_points();
  setup_survey_points();
  get_closest_relative_height();
  
  // dummy previous_time variables so that the first iteration doesn't blow up. 
  previous_time = millis();
  last_height_checked_time = previous_time; 
  last_sd_write_time = previous_time;

  pitch_servo.write(stow_angle);
  delay(1.5 * servo_delay);
  }

//**********************************************************************************//
//**********************************************************************************//
void loop() {
  // To debug this code, turn line 276 to open and comment out line 277
  // Turn line 280 to < 0 instead of 3 and comment out all of 290 to 302
  
  // reading the relay signal from Ardupilot
  int relay_cmd = 1; 
//  int relay_cmd = digitalRead(relay_pin);
  
  // if there is new data and it has a good fix
  if (gnss.Read() && (gnss.fix() < 0)) {

    // get current position and calculate the target gimbal roll angle every frame
    Serial.println("Calling position.......");
    get_current_pos();
    get_target_roll_angle();
    get_target_pitch_angle();
    }

  // This step can take up to 10 minutes.
//  else if (gnss.fix() < 3){
//    Serial.println("Waiting for GNSS fix.");
//    Serial.print(gnss.fix());
//    Serial.print("\t");
//    Serial.print(gnss.num_sv());
//    Serial.print("\t");
//    Serial.print(gnss.lat_deg(), 6);
//    Serial.print("\t");
//    Serial.print(gnss.lon_deg(), 6);
//    Serial.print("\t");
//    Serial.print(gnss.alt_wgs84_m(), 2);
//    Serial.print("\n");
//  }

  // Logic to take care of relay fluctuations. Check for consistency for certain iterations 
  if (!_relay_state) {
    if (relay_cmd == 1) {
      _state_counter++;
      }    
    else {
      _state_counter = 0;
      }
    } 
  else if (_relay_state) {
    if (relay_cmd == 0) {
      _state_counter++;
      }
    else {
      _state_counter = 0;
      }
    }

  // if relay_cmd is changed and consistent for certain number of times, then change relay state. 
  if (_state_counter > _relay_threshold) {
    _state_counter = 0;
    _relay_state = !_relay_state;
    }

  // check the closest relative height at certain intervals 
  if ((millis() - last_height_checked_time) > HEIGHT_CHECK_PERIOD_MS) {
    Serial.println("Checking for the next closest point");
    get_closest_relative_height();
    last_height_checked_time = millis();
    }

  // do nothing if there's no data on IMU
  if (!dmpReady) return;

  // if the relay state is ON
  if (_relay_state) {
    if (!trans_comp){
      roll_servo.write(_servo_center_angle);;
      pitch_servo.write(_servo_center_angle);;
      trans_comp = true;
    }
    else{
    // initial movement from center position to the target roll angle
    if (_servo_centered) {

      // calculate the required servo cmd required for target roll angle
      float servo_cmd_roll = calculate_servo_cmd_roll(target_roll_angle);
      goto_servo_angle_slowly_roll(servo_cmd_roll);
      _servo_centered = false; 

      float servo_cmd_pitch = calculate_servo_cmd_pitch(target_pitch_angle);
      goto_servo_angle_slowly_pitch(servo_cmd_pitch);
      _servo_centered = false;
      }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      get_IMU_state();
      
      // calculates target euler angles from target point's position vector
      get_target_roll_angle();
      get_target_pitch_angle();
      
      // get current euler angles
      float current_roll_angle = get_gimbal_state_roll();  
      float current_pitch_angle = get_gimbal_state_pitch(); 
  
      // Angular rate controller
      float k_out = compute_controller_output_roll(current_roll_angle);
      float p_out = compute_controller_output_pitch(current_pitch_angle);
  
      // Commanded angle output to the servos
      write_to_servo_roll(k_out); 
      write_to_servo_pitch(p_out);    
      }
     }
    }

  // when the relay state is OFF
  else {

    previous_time = millis();

    // if the servo is not centered
    if (!_servo_centered) {
      
      // centering the servos. 
      Serial.println("Centering the servos......");
      goto_servo_angle_slowly_roll(_servo_center_angle);
      pitch_servo.write(stow_angle);
      _servo_centered = true;
      trans_comp = false;
      } 
    }
  
  // Write data to SD Card periodically 
  if (new_file_created && (millis() - last_sd_write_time > SD_CARD_PERIOD_MS)){
    File file = SD.open(file_name.c_str(),FILE_WRITE);
    last_sd_write_time = millis();
    if (file){
      file.seek(EOF);
      String data_string = String(gnss.utc_year())+","+String(gnss.utc_month())+","+String(gnss.utc_day())+","+
      String(gnss.utc_hour())+","+String(gnss.utc_min())+","+String(gnss.utc_sec())+","+String(gnss.utc_nano())+","+
      String(gnss.lat_deg(),7)+","+String(gnss.lon_deg(),7)+","+String(target_roll_angle,2)+","+String(ypr[2],2)+","+
      String(target_pitch_angle,2)+","+String(ypr[1],2)+","+String(cum_error_pitch,2);
      file.println(data_string.c_str());
      file.close();
      }
    else{
      Serial.println ("SD card open failed");
      }
    }
  }

//**********************************************************************************//
float get_gimbal_state_roll(){
  // Based on the MPU_6050 library example
  // Getting Euler Angles
 
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // converting the roll angle to radians. Negative sign depends on the mounting of the IMU. Check before setup.
  float roll_angle = ypr[2] * - r2d;
  
  if (_DEBUG) {
    Serial.print("roll_angle\t");
    Serial.println(roll_angle);
  }

  return roll_angle;
}

//**********************************************************************************//
float get_gimbal_state_pitch(){
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch_angle = ypr[1] * -r2d;
  
  if (_DEBUG) {
    Serial.print("pitch_angle\t");
    Serial.println(pitch_angle);
  }

  return pitch_angle;
}

//**********************************************************************************//
void get_target_roll_angle() {
  // current normal distance from the line-of-interest
  float current_distance_from_line = current_pos.distance_from_origin * sin(current_pos.deviation_from_line * d2r);

  // Constant roll angle of 0
  target_roll_angle = 73;
  
//  target_roll_angle = max(min_roll_angle, min(target_roll_angle, max_roll_angle)); 
 

  if (_DEBUG) {
    Serial.println("-----------------------");
    Serial.print("Current distance from line:: ");
    Serial.print(current_distance_from_line);
    Serial.println(" m");
    
    Serial.println("-----------------------");
    Serial.print("Target Roll Angle:: ");
    Serial.print(target_roll_angle);
    Serial.println(" deg");
    }    
  }

//**********************************************************************************//
void get_target_pitch_angle() {
  // required roll angle based on normal distance and height

  // Constant pitch angle of 0
    target_pitch_angle = 0;

  if (_DEBUG) { 
    Serial.println("-----------------------");
    Serial.print("Target Pitch Angle:: ");
    Serial.print(target_pitch_angle);
    Serial.println(" deg");
    }    
  }

//**********************************************************************************//
float compute_controller_output_roll(float current_roll_angle) {
  // PI controller with Feedback
  
  current_time = millis();
  unsigned long elapsed_time = (unsigned long)(current_time - previous_time);

  // error on reference
  float error_roll = target_roll_angle - current_roll_angle;

  // cumulative error
  cum_error_roll += error_roll * elapsed_time / 1000;
    
  if (_DEBUG) {
    Serial.print("Error_roll:: \t");
    Serial.println(error_roll);
    Serial.print("Cumulative error roll:: \t");
    Serial.println(cum_error_roll);
  }

  // PI controller output (Angular roll rate cmd)
  float output_roll_rate = Kp * error_roll + Ki * cum_error_roll;

  if (_DEBUG) {
    Serial.print("Output angular rate:: \t");
    Serial.println(output_roll_rate);
  }

  // integrating the angular roll rate
  control_roll_out += output_roll_rate * elapsed_time / 1000;
  //previous_time = millis();

  return control_roll_out;
  }

//**********************************************************************************//
float compute_controller_output_pitch(float current_pitch_angle) {
  // PI controller with Feedback
  //current_time = millis();
  unsigned long elapsed_time = (unsigned long)(current_time - previous_time);

  // error on reference
  float error_pitch = target_pitch_angle - current_pitch_angle;

  // cumulative error
  cum_error_pitch += error_pitch * elapsed_time / 1000;
    
  if (_DEBUG) {
    Serial.print("Error_pitch:: \t");
    Serial.println(error_pitch);
    Serial.print("Cumulative error pitch:: \t");
    Serial.println(cum_error_pitch);
  }

  // PI controller output (Angular roll rate cmd)
  float output_pitch_rate = Kp * error_pitch;

  if (_DEBUG) {
    Serial.print("Output pitch angular rate:: \t");
    Serial.println(output_pitch_rate);
  }

  // integrating the angular roll rate
  control_pitch_out += output_pitch_rate * elapsed_time / 1000;
  if (_DEBUG) {
    Serial.print("Controller pitch angle output pre_sat: \t");
    Serial.println(control_pitch_out);
  }
  control_pitch_out = max(min_pitch_angle, min(control_pitch_out, max_pitch_angle));

  previous_time = millis();

  if (_DEBUG) {
    Serial.print("Controller pitch angle output: \t");
    Serial.println(control_pitch_out);
  }
  
  return control_pitch_out;
  }

//**********************************************************************************//
float calculate_servo_cmd_roll(float roll_cmd) {
  
  // mapping from required roll angle to required servo angle. 
  float slope_roll = (servo_roll_max - servo_roll_min) / (max_roll_angle);
  
  // saturate the calculated angle
  return saturate_servo_cmd_roll(slope_roll * roll_cmd + _servo_center_angle);
  }

//**********************************************************************************//
float calculate_servo_cmd_pitch(float pitch_cmd) {
  
  // mapping from required roll angle to required servo angle. 
  float slope_pitch = -(servo_pitch_max - servo_pitch_min) / (max_pitch_angle);
  
  // saturate the calculated angle
  return saturate_servo_cmd_pitch(slope_pitch * pitch_cmd + _servo_center_angle);
  }

//**********************************************************************************//
float saturate_servo_cmd_roll(float cmd_servo_angle_roll){
  // Saturating the servo cmd 
  if (cmd_servo_angle_roll < servo_roll_max) {
    cmd_servo_angle_roll = servo_roll_max;
    }
  else if (cmd_servo_angle_roll > servo_roll_min) {
    cmd_servo_angle_roll = servo_roll_min;
    }
  return cmd_servo_angle_roll;
  }

//**********************************************************************************//
float saturate_servo_cmd_pitch(float cmd_servo_angle_pitch){
  // Saturating the servo cmd
  if (cmd_servo_angle_pitch < servo_pitch_max) {
    cmd_servo_angle_pitch = servo_pitch_max;
    }
  else if (cmd_servo_angle_pitch > servo_pitch_min) {
    cmd_servo_angle_pitch = servo_pitch_min;
    }
  return cmd_servo_angle_pitch;
  }

//**********************************************************************************//
void goto_servo_angle_slowly_roll(float servo_cmd_roll) {
  int current_cmd_roll = last_servo_cmd_roll;

  if (servo_cmd_roll > current_cmd_roll) {
    for ( ; current_cmd_roll <= servo_cmd_roll; current_cmd_roll++) {
      roll_servo.write(current_cmd_roll);
      delay(1.5 * servo_delay);
      }
    last_servo_cmd_roll = servo_cmd_roll;
    }
  else if (servo_cmd_roll < current_cmd_roll) {
    for ( ; current_cmd_roll >= servo_cmd_roll; current_cmd_roll--) {
      roll_servo.write(current_cmd_roll);
      delay(1.5 * servo_delay);
      }
    last_servo_cmd_roll = servo_cmd_roll;  
    }
    
  // dummy previous_time variables so that the iteration after slow movement doesn't blow up 
  previous_time = millis();
  last_height_checked_time = previous_time; 
  last_sd_write_time = previous_time;
  }

//**********************************************************************************//
void goto_servo_angle_slowly_pitch(float servo_cmd_pitch) {
  int current_cmd_pitch = last_servo_cmd_pitch;

  if (servo_cmd_pitch > current_cmd_pitch) {
    for ( ; current_cmd_pitch <= servo_cmd_pitch; current_cmd_pitch++) {
      pitch_servo.write(180 - current_cmd_pitch);
      delay(1.5 * servo_delay);
      }
    last_servo_cmd_pitch = servo_cmd_pitch;
    }
  else if (servo_cmd_pitch < current_cmd_pitch) {
    for ( ; current_cmd_pitch >= servo_cmd_pitch; current_cmd_pitch--) {
      pitch_servo.write(180 - current_cmd_pitch);
      delay(1.5 * servo_delay);
      }
    last_servo_cmd_pitch = servo_cmd_pitch;  
    }
    
  // dummy previous_time variables so that the iteration after slow movement doesn't blow up 
  previous_time = millis();
  last_height_checked_time = previous_time; 
  last_sd_write_time = previous_time;
  }

//**********************************************************************************//
void write_to_servo_roll(float control_roll_out){

  // mapping the required servo cmd based on roll angle cmd
  float servo_cmd_roll = calculate_servo_cmd_roll(control_roll_out);
  
  if (_DEBUG) {
    Serial.print("Roll servo cmd:: \t");
    Serial.println(servo_cmd_roll);
  }
  
  // sending angle command to servo
  roll_servo.write(servo_cmd_roll);
  last_servo_cmd_roll = servo_cmd_roll;
  _servo_centered = false;
}  

//**********************************************************************************//
void write_to_servo_pitch(float control_pitch_out){

  // mapping the required servo cmd based on roll angle cmd
  float servo_cmd_pitch = calculate_servo_cmd_pitch(control_pitch_out);
  
  if (_DEBUG) {
    Serial.print("Pitch servo cmd:: \t");
    Serial.println(servo_cmd_pitch);
  }
  
  // sending angle command to servo
  pitch_servo.write(servo_cmd_pitch);
  last_servo_cmd_pitch = servo_cmd_pitch;
  _servo_centered = false;
}  
//**********************************************************************************//
void get_IMU_state(){
 if (_DEBUG) {
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
}
