# ArduinoDUE-Interfacing-BMI270

Microcontroller used : Arduino DUE

Important :
  1. I have not used [SparkFun_BMI270_Arduino_Library](https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library) [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) or  because I assumed we do not have SparkFun BMI270 6DoF IMU Breakout or Arduino Nano 33 BLE Sense Rev2 but rather only a standalone working BMI270 IMU. I have used the API provided directly by Bosh Sensor Tech [BMI270_Sensor_API](https://github.com/boschsensortec/BMI270_SensorAPI) to connect to the BMI270 IMU.

  2. Due to lack of avability of BMI270 I cannot verify the working of the code on actual hardware setup. I have verified the working of sensor data processing by making some fake sensor data and performing state calculations on it. This is available in the file ______. For the final file I have referred to many examples to prevent against any errors. I also couldn't find a BMI270 Arduino Interface Simulator. 

  3. I have assumed the accelerometer of the sensor is located at the center of mass of the system. Thus accelerometer only measures linear acceleration and gyroscope readings can be used to get orientation.

  4. Accelerometer values can be used along with gyroscope to give more accurate estimation of orientation. However this only works when there isn't linear motion since we use gravity to calculate the angle change. I have assumed there is enough linear acceleration to make readings of accelerometer for orientation calculation incorrect.
  
  5. I have explained parts of the code below. Refer to comments given in code for in-depth understanding.

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
  2. Gyroscope sensor data is in int16_t with range corresponding to +/- 2000 dps. This is configured using  . This reading is converted to rad/s using the following conversion:
     ```
      ((float)int16_value / 32768.0) * 2000.0 * (PI / 180.0) 
     ```

## Part 3 : Audio and Visual Representation Of States
