#ifndef APP_H
#define APP_H

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>
#include <Adafruit_MAX1704X.h>
#include <BleGamepad.h>
#include <stdint.h>
#include <Adafruit_AHRS_NXPFusion.h> 


#define CALIBRATION_ADDRESS 0x100
#define SETTING_ADDRESS 0x00
#define SETTINGS_VALID_FLAG 0xAA55

#define IMU1_INT_PIN 40
#define IMU2_INT_PIN 38


#define FUEL_GAUGE_I2C_SDA_PIN 34
#define FUEL_GAUGE_I2C_SCL_PIN 33

#define VSVY_EN_PIN 18

#define numOfButtons 64
#define numOfHatSwitches 4 

#define LED_GREEN_PIN 37 
#define LED_RED_PIN 35 
#define LED_BLUE_PIN 36
#define MOTOR_EN_PIN 13 

#define FILTER_UPDATE_RATE_HZ 12.5

/*
brief Structure to hold IMU sensor groups
*/

struct SensorGroupIMU {
  Adafruit_LSM6DS3TRC* lsm6ds1 ;
  Adafruit_LSM6DS3TRC* lsm6ds2 ;
  Adafruit_LIS3MDL* lis3mdl1 ;
  Adafruit_LIS3MDL* lis3mdl2 ;
};


/*
brief Structure to hold configuration data for IMU and Joystick
*/
typedef struct __attribute__((packed)) config_IMU__Joystick {
  uint8_t IMU1_accel_gyro_rate;
  uint8_t IMU1_mag_freq;
  uint8_t IMU2_accel_gyro_freq;
  uint8_t IMU2_mag_freq;
  uint8_t IMU1_accel_range;
  uint8_t IMU1_gyro_range;
  uint8_t IMU1_mag_range;
  uint8_t IMU2_accel_range;
  uint8_t IMU2_gyro_range;
  uint8_t IMU2_mag_range;
  uint16_t Joystick_flex_sensor_rate;
} IMU_config_data_anJoystick;

typedef union __attribute__((packed)) {
  IMU_config_data_anJoystick configDataIMUJOTISK;
  uint8_t rawData[15];
} IMU_config_data_anJoystick_packed;

typedef struct __attribute__((packed)) Overall_status {
    uint8_t status_code ; //       0: no error     1: General Error occure
    uint8_t Fuelgause_status ;  // 0: not detect   1: failed   2: idle  3: running
    uint8_t Imu1_status ;  //      0: not detect   1: failed   2: idle  3: running
    uint8_t Imu2_status ;  //      0: not detect   1: failed   2: idle  3: running 

} Overall_status_data; 


typedef union __attribute__((packed)) {
    Overall_status_data overallStatusData;
    uint8_t rawData[4];

} Overall_status_data_packed;


enum statuscode_sensor {
    NOT_DETECT = 0 ,
    FAILED = 1 ,
    IDEL = 2 ,
    RUNNING = 3 

} ;


 enum  statuscode {
    NO_ERROR = 0 ,
    GENERAL_ERROR = 1 
 }   ;

 typedef struct __attribute__((packed)) {
   
    int16_t accelX ;
    int16_t accelY ;
    int16_t accelZ ; 
    int16_t GyroX ;
    int16_t GyroY ; 
    int16_t GyroZ ; 
    int16_t MagX ;
    int16_t MagY ;  
    int16_t MagZ ;
    
 } IMU1_data_Raw ;

 typedef union __attribute__((packed)) {
   IMU1_data_Raw data_Imu1 ; 
   uint8_t rawData[18]; 
 } IMU1_data_Raw_packed;



 typedef struct __attribute__((packed)) {
   
    int16_t accelX ;
    int16_t accelY ;
    int16_t accelZ ; 
    int16_t GyroX ;
    int16_t GyroY ; 
    int16_t GyroZ ; 
    int16_t MagX ;
    int16_t MagY ;  
    int16_t MagZ ;
    
 } IMU2_data_Raw ;

 typedef union __attribute__((packed)) {
   IMU2_data_Raw data_Imu2 ; 
   uint8_t rawData[18]; 
 } IMU2_data_Raw_packed;



typedef struct __attribute__((packed)) {

    float EulerYaw; 
    float EulerPitch;
    float EulerRoll;
    uint8_t calibation ; 


} IMU1_euler_calib_status ;

typedef union __attribute__((packed)) {
    IMU1_euler_calib_status eulerCalibStatus; 
    uint8_t rawData[13]; 

} IMU1_euler_calib_status_packed;



typedef struct __attribute__((packed)) {

    float EulerYaw; 
    float EulerPitch;
    float EulerRoll;
    uint8_t calibation ; 


} IMU2_euler_calib_status ;

typedef union __attribute__((packed)) {
    IMU2_euler_calib_status eulerCalibStatus; 
    uint8_t rawData[13]; 

} IMU2_euler_calib_status_packed;

typedef struct __attribute__((packed)) {
    uint8_t calibationIMU1; 
    uint8_t calibationIMU2  ; 

} calibratee;




typedef struct __attribute__((packed)) {
  
    float flex_sensor_1 ; 
    float flex_sensor_2 ; 
    float flex_sensor_3 ;
    float flex_sensor_4 ;
    float flex_sensor_5 ;
    
} Flex_sensor_data; 




#endif
