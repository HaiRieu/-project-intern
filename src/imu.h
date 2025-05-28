#ifndef IMU_H
#define IMU_H

#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>
#include "app.h"

#define IMU1_INT_PIN 40
#define IMU2_INT_PIN 38

extern Adafruit_LSM6DS3TRC lsm6ds1;
extern Adafruit_LSM6DS3TRC lsm6ds2;
extern Adafruit_LIS3MDL lis3mdl1;
extern Adafruit_LIS3MDL lis3mdl2;
extern Adafruit_Sensor_Calibration_EEPROM cal;

extern volatile bool IMUsAvailable;
extern volatile bool imuDataReady;
extern bool calibrationLoaded;

bool restoreSettings();
void setDefaultSettings();
bool initIMU();
void setupIMUDataRate();
void setupIMUInterrupts();
void IRAM_ATTR imu1InterruptHandler();
bool loadCalibration();

#endif
