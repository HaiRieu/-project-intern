#ifndef IMU_H
#define IMU_H
#include "app.h"

bool restoreSettings( );
void setDefaultSettings();
bool initIMU();
void setupIMUDataRate();
void setupIMUInterrupts();
void IRAM_ATTR imu1InterruptHandler();
bool loadCalibration();
void setupBLEGamepad() ; 

#endif
