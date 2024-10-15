# ArduinoDUE-Interfacing-BMI270

Microcontroller used : Arduino DUE

Important :
  1. [SparkFun_BMI270_Arduino_Library](https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library) or [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) have not been used based on the assumption that we do not have SparkFun BMI270 6DoF IMU Breakout or Arduino Nano 33 BLE Sense Rev2 but rather only a standalone working BMI270 IMU. The API provided directly by Bosh Sensor Tech [BMI270_Sensor_API](https://github.com/boschsensortec/BMI270_SensorAPI) to connect to the BMI270 IMU has been used.

  2. Due to lack of avability of BMI270 the code cannot be verified on actual hardware setup. The working of sensor data processing part of code has been verified by making some fake sensor data and performing state calculations on it. This is available in the file ______. For the final file many examples have been referred to prevent against any errors. A BMI270 Arduino Interface Simulator also couldn't be found. 

  3. The accelerometer sensor has been assumed to be located at the center of mass of the system. Thus accelerometer only measures linear acceleration and gyroscope readings can be used to get orientation.

  4. Accelerometer values can be used along with gyroscope to give more accurate estimation of orientation. However this only works when there isn't linear motion since we use gravity to calculate the angle change. Enough linear acceleration has been assumed to make readings of accelerometer for orientation calculation incorrect.
  
  5. Some parts of the code have been explained below. Refer to comments given in code for in-depth understanding.

Here is the schematic diagram for the connections :


## Part 1 : Connecting To BMI270

This is done using [BMI270_Sensor_API](https://github.com/boschsensortec/BMI270_SensorAPI) library provided by Bosh Sensor Tech. 
Parameters used :
  1. Default config file of BMI270 given in the API library is used.
  2. Accelerometer and Gyroscope are configured to take sensor readings every 100Hz. This is defined by values BMI2_ACC_ODR_100HZ and BMI2_GYR_ODR_100HZ.
  3. SPI protocol is used to communicate between BMI270 and Arduino DUE.
  4. Interrupt of BMI270 although defined are not used, rather sensor reading is taken every 100Hz using SPI.
  5. Sensor readings of accelerometer and gyroscope are taken every 100Hz. This is defined in the global variable data_acquire_freq.

## Part 2 : Calculating States from Sensor Data

Module Working :
  1. Accelerometer sensor data is in int16_t with range corresponding to +/- 2G. This is configured using BMI2_ACC_RANGE_2G. This reading is converted to m/s^2 using the following conversion:
     ```
     ((float)int16_value / 32768.0) * 19.62
     ```
     
  2. Gyroscope sensor data is in int16_t with range corresponding to +/- 2000 dps. This is configured using BMI2_GYR_RANGE_2000. This reading is converted to rad/s using the following conversion:
     ```
      ((float)int16_value / 32768.0) * 2000.0 * (PI / 180.0) 
     ```
     
  3. Using gyroscope data, euler angle of rotations are calculated.
     ```
       angles[0] += gyro_data.x * delta_time;  // Roll
       angles[1] += gyro_data.y * delta_time;  // Pitch
       angles[2] += gyro_data.z * delta_time;  // Yaw
     ```
     
  4. These euler angles are inverted and converted to quaternions (used for 3D orientation and rotations). Using these quaternions we can transform the accelerometer readings for the current orientation back to the x,y,z values for the inertial or starting orientation. Maths for the algorithm are described here : [Quaternions](https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html).
     ```
      angles[0] = -angles[0];
      angles[1] = -angles[1];
      angles[2] = -angles[2];
      euler_to_quaternion(angles, q);
      rotate_vector(q, accel_data[i], adjusted_accel);
     ```

  5. Acceleration values in intertial or starting frame coordinate system are now corrected by subracting gravity and passing thorugh a low pass filter with parameter alpha.
     
  6. True acceleration values are now used to calculate displacement, velocity and jerk. The thresholds for each states are defined as:
     ```
      float displacement_threshold = 1; // Threshold for displacement (m)
      float motion_threshold = 0.05; // Threshold for motion (m/s)
      float jerk_threshold = 3; // Threshold for high jerk (m/s^3)
     ```
     
  7. Based on these values final states are calculated.
     
      ```
      if (current_state == "Stationary") {
        State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
      } else if (current_state == "Motion") {
        State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
      } else if (current_state == "High Jerk") {
        State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
      } else if (current_state == "Displaced") {
        State_Signal(current_state, prev_state, prev_millis, current_millis, ledState);
      }
      ```


## Part 3 : Audio and Visual Representation Of States

To make non-blocking code (allow peripherals like LED and buzzer to run alongside processing code) I have used millis() function for making LED PWN signal and tone() to transmit square wave for buzzer.
2 Switches are also provided. SW1 starts the system. SW2 stops the systems and resets it.

Representation Of States :

  1. **Stationary** : RED LED connected is blinked at every 500ms (defined by interval_normal variable).
     ```
     if (current_state == "Stationary"){
        if (current_millis - prev_millis >= interval_normal) {
          prev_millis = current_millis;
          
          // Toggle the LED state
          ledState = !ledState;
          digitalWrite(redLedPin, ledState ? HIGH : LOW);       // For stationary state, toggle the RED Led
        }
     }
     ```
     
  2. **Motion** : BLUE LED connected is blinked at every 500ms (defined by interval_normal variable).
     ```
      else if (current_state == "Motion") {
          if (current_millis - prev_millis >= interval_normal) {
            prev_millis = current_millis;
            
            // Toggle the LED state
            ledState = !ledState;
            digitalWrite(blueLedPin, ledState ? HIGH : LOW);       // For motion state, toggle the BLUE Led
          }
        }
     ```
     
  3. **High Jerk** : All LEDs connected are blinked at every 100ms (defined by interval_alert variable) and audio sound of 1000Hz is made by the connected buzzer for 200ms.
     ```
      else if (current_state == "High Jerk") {
          if (current_millis - prev_millis >= interval_alert) {
            // Toggle the LED state
            ledState = !ledState;
            digitalWrite(greenLedPin, ledState ? HIGH : LOW);  // For high jerk state, toggle all Leds
            digitalWrite(blueLedPin, ledState ? HIGH : LOW);
            digitalWrite(redLedPin, ledState ? HIGH : LOW);
            if (current_millis - prev_millis >= interval_buzz){
              tone(buzzerPin, 1000, 200);   // Make a 1000Hz tone for 200ms
            }
            prev_millis = current_millis;
          }
      }
     ```
     
  4. **Displacement Of More Than 1m** : GREEN LED connected constantly ON and audio sound of 10000Hz is made by the connected buzzer for 200ms.
     ```
      else if (current_state == "Displaced") {
          if (current_millis - prev_millis >= interval_alert) {
            // Set LED High
            digitalWrite(greenLedPin, HIGH);  // For displaced by more than 1m state, make the GREEN Led HIGH
            if (current_millis - prev_millis >= interval_buzz){
              tone(buzzerPin, 10000, 200);   // Make a 10000Hz tone for 200ms
            }
            prev_millis = current_millis;
          }
        }
     ```

