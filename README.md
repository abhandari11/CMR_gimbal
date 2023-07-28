# CMR_gimbal
Gimbal Control and Test Codebase for Teensy4.1 for Bistatic Radar Flights

Relevant included files contain:
Teensy codes > controller > gimbal_control_bistatic
    - Reads lat-long-alt location and tracks the gimbal accordingly for 
      roll servo and holds the pitch servo at a 0 deg. angle.

Teensy codes > controller > gimbal_control_NADIR
    - Reads IMU data for the servos and keeps them both locked at a 0 deg. (Inputted
      90 deg. servo angle) and keeps them idle (NADIR).
      
Teensy codes > imu > IMUTesting
    - Tracks the data in real time for the IMU, so both servos (roll and pitch) 
      can be tested and correlate with the IMU.