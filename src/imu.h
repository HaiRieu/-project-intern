#ifndef IMU_H
#define IMU_H

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h> 
#include "app.h" 

#define IMU1_INIT_pin 40 
#define IMU2_INIT_pin 38 


extern Adafruit_LSM6DS3TRC lsm6ds1;
extern Adafruit_LSM6DS3TRC lsm6ds2;
extern Adafruit_LIS3MDL lis3mdl1;
extern Adafruit_LIS3MDL lis3mdl2;
extern Adafruit_Sensor_Calibration cal; 


extern bool IMUsAvailable;
extern volatile bool imuDataReady = false ; 


bool initIMU();
void setupIMUdataRate();
void setupIMUInterrupts(); 
void IRAM_ATTR imu1InterruptHandler(); 
bool LoadCallibration() ;




#endif 