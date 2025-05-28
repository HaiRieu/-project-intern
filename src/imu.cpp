#include "imu.h"
#include "app.h"

Adafruit_LSM6DS3TRC lsm6ds1;
Adafruit_LSM6DS3TRC lsm6ds2;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LIS3MDL lis3mdl2;

bool IMUsAvailable = false;
bool calibarationLoaded = false; 

Overall_status_data_packed overallStatusDatapPacked;
IMU1_euler_calib_status_packed imu1EulerCalibration ; 
IMU2_euler_calib_status_packed imu2EulerCalibration ;

#define CALIBRATION_ADDRESS 0x100  


/*

brief : 
IMU start and check for errors


*/


bool initIMU()
{  

  if (!lsm6ds1.begin_I2C(0x6A))
  {
    Serial.println("Failed to find LSM6DS1 chip");
    overallStatusDatapPacked.overallStatusData.status_code =  statuscode::GENERAL_ERROR ;  
    overallStatusDatapPacked.overallStatusData.Imu1_status =  statuscode_sensor::FAILED ; 
    
    IMUsAvailable = false;
    return false;
  }
  if (!lsm6ds2.begin_I2C(0x6B))
  {
    Serial.println("Failed to find LSM6DS2 chip");
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR  ; 
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED  ; 
    IMUsAvailable = false;
    return false;
  }

  if (!lis3mdl1.begin_I2C(0x1E))
  {
    Serial.println("Failed to find LIS3MDL chip 1");
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR ; 
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED  ; 
    IMUsAvailable = false;
    return false;
  }

  if (!lis3mdl2.begin_I2C(0x1C))
  {
    Serial.println("Failed to find LIS3MDL chip 2");
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR ; 
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED  ; 
    IMUsAvailable = false;
    return false;
  }
  Serial.println("IMU chips initialized successfully");
  overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
  overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
  IMUsAvailable = true;
  return true;
}



/*
brief :
Setup the IMU data rate and range for both accelerometer and gyroscope.

*/

void setupIMUdataRate() {
  lsm6ds1.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds1.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds1.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  lis3mdl1.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl1.setRange(LIS3MDL_RANGE_4_GAUSS);

  lsm6ds2.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds2.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds2.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  lis3mdl2.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl2.setRange(LIS3MDL_RANGE_4_GAUSS);
}



/*
brief :
Setup the IMU interrupts for both IMUs. This function configures the interrupt pins and attaches the interrupt handler. 
*/
void setupIMUInterrupts() {
  pinMode(IMU1_INIT_pin , INPUT )  ; 
  pinMode(IMU2_INIT_pin , INPUT )  ;

  attachInterrupt(digitalPinToInterrupt(IMU1_INIT_pin) , imu1InterruptHandler , RISING) ;  
    lsm6ds1.configInt1(false, false, true); 
    lsm6ds2.configInt1(false, false, true); 

     Serial.println("IMU interrupts configured");
}

void IRAM_ATTR imu1InterruptHandler() {
   imuDataReady = true ; 
}



/* 
brief :
      Load the calibration data from EEPROM 
*/
bool LoadCallibration() {
    if(!EEPROM.begin(512)){
       Serial.println("Failed to initialize EEPROM "); 
       return false ;
    }

    EEPROM.get(CALIBRATION_ADDRESS, imu1EulerCalibration) ; 
    EEPROM.get(CALIBRATION_ADDRESS+ sizeof(IMU1_euler_calib_status_packed) , imu2EulerCalibration) ;

  if(imu1EulerCalibration.eulerCalibStatus.calibation != 0 && imu2EulerCalibration.eulerCalibStatus.calibation != 0 ) {

        calibarationLoaded = true ;
        imu1EulerCalibration.eulerCalibStatus.calibation = 1 ;
        imu2EulerCalibration.eulerCalibStatus.calibation = 1 ;  
        return true ; 
  } 

  calibarationLoaded = false ; 
  imu1EulerCalibration.eulerCalibStatus.calibation = 0 ;    
  imu2EulerCalibration.eulerCalibStatus.calibation = 0 ;
  return false; 
}