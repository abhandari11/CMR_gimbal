//#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "ubx.h"
#include <SPI.h>
#include <SD.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//****Tunable Parameters****//

// Controller gains
float Kp = 3.0; // 6.0 
float Kd = 0.0;
float Ki = 0.0; // 1.5;

// range of Servo movements
float max_roll_angle = 75;
float min_roll_angle = -20;

int _servo_center_angle = 130;
float servo_max = 55;
float servo_min = 155;

float servo_delay = 10;

float max_angular_rate = 500;  

// gnss baud rate 
int _gnss_baud = 115200;

// update rate: 10 ms
const int16_t LOOP_PERIOD_MS = 10;

// rate to check closest survey point
const int16_t HEIGHT_CHECK_PERIOD_MS = 10000;

bool _DEBUG = false;

int _relay_threshold = 10;

// write new data to sd card at 5Hz
uint8_t SD_CARD_PERIOD_MS = 250;    


//********//

bool new_file_created = false;
String file_name = "";
unsigned long last_sd_write_time;

struct PositionGeo {
  double latitude;
  double longitude;
  double alt;
  double dist_from_origin;
  double dev_from_line;
  };

// change the number of elements in the struct 
struct PositionGeo gnd_points[2];

//********//

const double r2d = 180 / M_PI;
const double d2r = 1 / r2d;

Servo roll_servo;
MPU6050 mpu;
bfs::Ubx gnss(&Serial2);

const int pin_roll_servo = 20;
const int relay_pin = 25;

bool _relay_state = false;
int _state_counter = 0;

bool _servo_centered = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];
float roll_angle;
float offset_roll_angle;

// target angle
double target_roll_angle;
double closest_rel_height;

// controller output
float output_roll_rate;


unsigned long last_height_checked_time;
unsigned long current_time, previous_time;
unsigned long elapsed_time;
float error_roll, previous_error_roll;
float cum_error_roll, rate_error_roll;
float control_out_roll;
double last_servo_cmd;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// variables for position
PositionGeo current_pos;

// length and bearing of the survey line
double line_bearing;
double line_length;


void setup() {

  // initialize serial communication
  Serial.begin(115200);

  // setting the digital relay switch
  // from the flight computer
  pinMode(relay_pin, INPUT);
  Serial.println("-----------------------");
  Serial.print("The relay is : ");
  Serial.print(digitalRead(relay_pin));

  // setting the servos to neutral point
  roll_servo.attach(pin_roll_servo);
  Serial.println("Centering the servos...");
  Serial.println("-----------------------");
  roll_servo.write(_servo_center_angle);
  last_servo_cmd = _servo_center_angle;

  // setting up I2C connection to IMU
  Serial.println("-----------------------");
  Serial.println("Setting up I2C connection...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // wait for ready:: Needs keyboard entry
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
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
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

  // Calculating some existing bias on final euler angles
  Serial.println("-----------------------");
  Serial.println("Calculating offsets for Euler angles...");
  calculate_offsets_angle();
  Serial.print("Roll offset is ");
  Serial.println(offset_roll_angle);

  // wait for ready:: Needs keyboard entry
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

  Serial.println("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while(1){}
    }
  Serial.println("SD card initialized");

  // Setting up the positioning stuff
  Serial.println("-----------------------");
  Serial.print("Setting up the ground survey points and GNSS...");
  gnss.Begin(115200);
  
  load_survey_points();
  Serial.println("Setting starting survey point as origin...");
 
  setup_survey_points();
  get_closest_relative_height();

  // dummy previous_time variable so that first iteration doesnt blow up
  previous_time = millis();
  last_height_checked_time = previous_time;
  last_sd_write_time = previous_time;
  
}

void load_survey_points() {
  Serial.print("Loading the survey points: Num=");
  Serial.println(sizeof(gnd_points) / sizeof(gnd_points[0]));

  // First Point
  // Set as Origin
  // 0, 1
  gnd_points[0].latitude = 33.2944701;
  gnd_points[0].longitude = -87.6392372;
  gnd_points[0].alt = 0;
  
  // Second Point
  gnd_points[1].latitude = 33.2945610;
  gnd_points[1].longitude = -87.6397942;
  gnd_points[1].alt = 0;  

/* 
Shelby quad prime
  0', 1', 2'
  // First Point
  // Set as Origin
  gnd_points[0].latitude = 33.21488;
  gnd_points[0].longitude = -87.54298;
  gnd_points[0].alt = 10;
  gnd_points[0].dist_from_origin = 0;
  
  // Second Point
  gnd_points[1].latitude = 33.21487;
  gnd_points[1].longitude = -87.54326;
  gnd_points[1].alt = 20;  

  // Last point
  gnd_points[2].latitude = 33.21492;
  gnd_points[2].longitude = -87.54358;
  gnd_points[2].alt = 30;  
*/

/*
// Arboretum parking lot
  // First Point
  // Set as Origin
  // 0, 1, 2
  gnd_points[0].latitude = 33.19428;
  gnd_points[0].longitude = -87.48189;
  gnd_points[0].alt = 10;
  gnd_points[0].dist_from_origin = 0;
  
  // Second Point
  gnd_points[1].latitude = 33.19441;
  gnd_points[1].longitude = -87.48190;
  gnd_points[1].alt = 20;  

  // Last point
  gnd_points[2].latitude = 33.19450;
  gnd_points[2].longitude = -87.48189;
  gnd_points[2].alt = 30;  
*/
  }

void setup_survey_points() {
  
  // getting distance from the origin for each survey point
  int n = sizeof(gnd_points) / sizeof(gnd_points[0]);
  
  for (int ii = 0; ii < n; ii++) {
    gnd_points[ii].dist_from_origin = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[ii].latitude, gnd_points[ii].longitude);
    gnd_points[ii].dev_from_line = 0.0;
    Serial.print("Dist of point from origin:: ");
    Serial.print(gnd_points[ii].dist_from_origin);
    Serial.println("m");
    }
  
  // getting the last index
  line_bearing = get_bearing(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[n-1].latitude, gnd_points[n-1].longitude);
  line_length = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, gnd_points[n-1].latitude, gnd_points[n-1].longitude);
  
  Serial.print("The length of the survey line is ");
  Serial.print(line_length);
  Serial.println("m");
  
  Serial.print("The bearing of the survey line is ");
  Serial.print(line_bearing);
  Serial.println(" deg");  
  }


void get_current_pos(){

  current_pos.latitude = gnss.lat_deg();
  current_pos.longitude = gnss.lon_deg();
  current_pos.alt = gnss.alt_wgs84_m();

  current_pos.dist_from_origin = get_distance(gnd_points[0].latitude, gnd_points[0].longitude, current_pos.latitude, current_pos.longitude);
  
  double current_bearing_from_origin = get_bearing(gnd_points[0].latitude, gnd_points[0].longitude, current_pos.latitude, current_pos.longitude);
  current_pos.dev_from_line = abs(line_bearing - current_bearing_from_origin);

  if (_DEBUG) {
  
    Serial.println("-----------------------");
    Serial.print("Current position:: \t");
    Serial.print(current_pos.latitude, 6);
    Serial.print("\t");
    Serial.print(current_pos.longitude, 6);
    Serial.print("\t");
    Serial.println(current_pos.alt, 3);
  
    Serial.println("-----------------------");
    Serial.print("Distance from origin :: \t");
    Serial.print(current_pos.dist_from_origin);
    Serial.println("m");
    
    Serial.print("Current Bearing from origin :: \t");
    Serial.print(current_bearing_from_origin);
    Serial.println("deg");
    
    Serial.print("Angle deviation from line :: \t");
    Serial.print(current_pos.dev_from_line);
    Serial.println("deg");
  }
}



void get_target_roll_angle() {
  // target_roll_angle = 20;

  double current_dist_from_line = abs(current_pos.dist_from_origin * sin(current_pos.dev_from_line * d2r));
  target_roll_angle = abs(r2d * atan2(current_dist_from_line, closest_rel_height));

  if (_DEBUG) {
    Serial.println("-----------------------");
    Serial.print("Current distance from line:: ");
    Serial.print(current_dist_from_line);
    Serial.println(" m");
    
    Serial.println("-----------------------");
    Serial.print("Target Roll Angle:: ");
    Serial.print(target_roll_angle);
    Serial.println(" deg");
    }  
  }


void get_closest_relative_height() {
  // call this function at a certain predefined rate
  
  double current_dist_along_line = current_pos.dist_from_origin * cos(current_pos.dev_from_line * d2r);
  int n = sizeof(gnd_points) / sizeof(gnd_points[0]);
  double diff_dist_along_line[n] = {};

  Serial.print("Current distance along the line:: ");
  Serial.println(current_dist_along_line);

  int index = 0;
  if (current_dist_along_line > 0) {
    
    for (int ii = 0; ii < n; ii++) {
      
      Serial.print("Ground point :: ");
      Serial.print(ii);
      Serial.print("\t");
      Serial.println(gnd_points[ii].dist_from_origin);
      
      diff_dist_along_line[ii] = abs(current_dist_along_line - gnd_points[ii].dist_from_origin);
      
      Serial.print("Distance from :: ");
      Serial.print(ii);
      Serial.print("\t");
      Serial.println(diff_dist_along_line[ii]);
      }
  
    // finding the index of minimum difference
    for(int i = 1; i < n; i++) {
        if(diff_dist_along_line[i] < diff_dist_along_line[index]) {
            index = i;
            }                
      }  
    }
    else {
      index = 0;
      Serial.println("Currently behind the survey line...");
      }
  
  // closest survey point to current position is gnd_points[index]
  // closest_rel_height = current_pos.alt - gnd_points[index].alt;
  
  // just hardcoding the closest relative height
  closest_rel_height = 10;
  
  Serial.println("-----------------------");
  Serial.print("Index of closest survey point:: ");
  Serial.println(index);
  Serial.print("Relative height from the closest survey point:: ");
  Serial.print(closest_rel_height);
  Serial.println("m");  
  }


static double get_distance(double lat1, double lon1, double lat2, double lon2) {
  // differences between lats and lons
  double dlat = (lat2 - lat1) * d2r;
  double dlon = (lon2 - lon1) * d2r;

  lat1 = lat1 * d2r;
  lat2 = lat2 * d2r;
  // Haversine Equations
  double a = pow(sin(dlat/2),2) + pow(sin(dlon/2), 2) * cos(lat1) * cos(lat2);
  double radius = 6378100;
  double c = 2 * asin(sqrt(a));
  return radius * c;
}


static double get_bearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = lat1 * d2r;
  lat2 = lat2 * d2r;
  lon1 = lon1 * d2r;
  lon2 = lon2 * d2r;

  // Haversine Equations
  double y = sin(lon2 - lon1) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
  double theta = atan2(y, x);
  double bearing = (int(theta*r2d) + 360) % 360;
  return bearing;
  }


void loop(){ 

  int relay_cmd = digitalRead(relay_pin);
  // int relay_cmd = 1;
  // unsigned long loop_start_time_ms = millis();

  if (gnss.Read() && (gnss.fix() > 2)) {
    get_current_pos();
    get_target_roll_angle();
    
    if (!new_file_created){
      new_file_created = true;
      file_name = String(gnss.utc_year())+String(gnss.utc_month())+String(gnss.utc_day())+String(gnss.utc_hour())+String(gnss.utc_min())+".txt";
      }
          
    }
  
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
 
  if ((millis() - last_height_checked_time) > HEIGHT_CHECK_PERIOD_MS) {
    Serial.println("Checking for the next closest point");
    
    // check closest relative height at a certain rate
    get_closest_relative_height();
    last_height_checked_time = millis();
    }

  if (_state_counter > _relay_threshold) {
    _state_counter = 0;
    _relay_state = !_relay_state;
    }
  
  if (!dmpReady) return;
  
  if (_relay_state) {    

    if (_servo_centered) {
      
      double servo_cmd = calculate_servo_cmd(target_roll_angle);
      servo_cmd = saturate_servo_cmd(servo_cmd);
      goto_servo_angle_slowly(servo_cmd);
      delay(500);
      previous_time = millis();
      _servo_centered = false;
      }
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      
      // calculates target euler angles from target point's position vector
      get_target_roll_angle();
      
      // get current euler angles
      get_gimbal_state();   
  
      // Angular rate controller
      compute_controller_output();
  
      // Commanded angle output to the servos
      write_to_servo();
      }    
    }
    
  else {
    previous_time = millis();
    
    if (!_servo_centered) {
        // setting the servos to neutral point
        Serial.println("Centering the servos...");
        Serial.println("-----------------------");
        goto_servo_angle_slowly(_servo_center_angle);
        _servo_centered = true;
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
      String(gnss.lat_deg(),7)+","+String(gnss.lon_deg(),7)+","+String(target_roll_angle,2)+","+
      String(roll_angle,2);
      file.println(data_string.c_str());
      file.close();
      }
    else{
      Serial.println ("SD card open failed");
      }
  
  }

}

void calculate_offsets_angle() {
  float roll_val = 0;
  int sample_num = 10;
  
  // taking simple average of sample_num measurements
  for (int i = 0; i < sample_num; i++) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll_val += (ypr[2] * -r2d);
    delay(100);
  }
  
  offset_roll_angle = roll_val / sample_num;
}


void get_gimbal_state(){
  // Based on the MPU_6050 library example
  // Getting Euler Angles
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  roll_angle = (ypr[2] * -r2d) - offset_roll_angle;
  
  if (_DEBUG) {
    Serial.print("roll_angle\t");
    Serial.println(roll_angle);
  }
}

void compute_controller_output(){
  
  current_time = millis();
  elapsed_time = (unsigned long)(current_time - previous_time);

  // Calculating error from feedback
  error_roll = target_roll_angle - roll_angle; 
  
  if (_DEBUG) {
    Serial.print("Error_roll:: \t");
    Serial.println(error_roll);
  }
  
  // Cumulative error
  cum_error_roll += error_roll * elapsed_time / 1000;

  // rate error
  rate_error_roll = (error_roll - previous_error_roll) * 1000 / elapsed_time;

  // PID output
  output_roll_rate = Kp * error_roll + Ki * cum_error_roll + Kd * rate_error_roll;

  // saturating the angular rate
  if (abs(output_roll_rate) > max_angular_rate) {
    if (output_roll_rate < 0) {
      output_roll_rate = -max_angular_rate;
    }
    else {
      output_roll_rate = max_angular_rate;
    }
  }

  if (_DEBUG) {
    Serial.print("Output angular rate:: \t");
    Serial.println(output_roll_rate);
  }
  
  // Integrating the angular rate for commanded angle
  control_out_roll += output_roll_rate * elapsed_time / 1000;
  
  if (_DEBUG) {
    Serial.print("Control output angle:: \t");
    Serial.println(control_out_roll);
  }
  
  previous_error_roll = error_roll;
  previous_time = current_time;
  
}


double calculate_servo_cmd(float roll_cmd) {
  double slope = (servo_max - servo_min) / (max_roll_angle);
  return slope * control_out_roll + _servo_center_angle;
  }

double saturate_servo_cmd(double servo_cmd) {
    // saturating servo command
  if (servo_cmd < servo_max) {
    servo_cmd = servo_max;
    }
  else if (servo_cmd > servo_min) {
    servo_cmd = servo_min;
    }
  return servo_cmd;
  }

  
void goto_servo_angle_slowly(double servo_cmd) {
  int current_cmd = last_servo_cmd;

  if (servo_cmd > current_cmd) {
    for ( ; current_cmd <= servo_cmd; current_cmd++) {
      roll_servo.write(current_cmd);
      delay(servo_delay);
      }
    last_servo_cmd = servo_cmd;
    }
  else if (servo_cmd < current_cmd) {
    for ( ; current_cmd >= servo_cmd; current_cmd--) {
      roll_servo.write(current_cmd);
      delay(servo_delay);
      }
    last_servo_cmd = servo_cmd;  
    }
  }

void write_to_servo(){
  double servo_cmd;
  
  // saturating commanded angle based on servo limits
  if (control_out_roll > max_roll_angle) {
    control_out_roll = max_roll_angle;
  }
  else if (control_out_roll < min_roll_angle) {
    control_out_roll = min_roll_angle;
  }

  servo_cmd = calculate_servo_cmd(control_out_roll);
  servo_cmd = saturate_servo_cmd(servo_cmd);
  
  if (_DEBUG) {
    Serial.print("servo cmd:: \t");
    Serial.println(servo_cmd);
    Serial.println("--------------");
  }
  
  // sending angle command to servo
  roll_servo.write(servo_cmd);
  last_servo_cmd = servo_cmd;
  _servo_centered = false;
}  
