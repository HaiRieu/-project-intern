#include <Arduino.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Calibration.h>
#include <EEPROM.h>
#include <Adafruit_MAX1704X.h>
#include "app.h"
#include "imu.h" 
#include "flueGaue.h" 
#include <BleGamepad.h>



#define VSVY_EN_PIN 18
#define SW_DET_PIN 6

#define SETTING_ADDRESS 0x00
#define SETTINGS_VAID_FLAG 0xAA55

Adafruit_LSM6DS3TRC lsm6ds1;
Adafruit_LSM6DS3TRC lsm6ds2;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LIS3MDL lis3mdl2;

Adafruit_MAX17048 fuelGauge;

Overall_status_data_packed overallStatusDatapPacked;
statuscode_sensor statusCodeSensor ;
statuscode statusCode; 

bool feulGaue = false;
bool IMUsAvailable = false;

/*
brief :
sets up the VSVY_EN_PIN and initializes the I2C
*/

void systemInit()
{
  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, LOW);
  Wire.begin(33, 34);
}

/*
brief :
Initializes the EEPROM and restores settings if available. 
*/
bool Restoresettings()
{
  if (EEPROM.begin(512))
  {
    IMU_config_data_anJoystick_packed configData;
    uint16_t validationFlag;

    EEPROM.get(SETTING_ADDRESS, validationFlag);
    if (validationFlag == SETTINGS_VAID_FLAG)
    {
      EEPROM.get(SETTING_ADDRESS + sizeof(validationFlag), configData);

      // Configure IMU1
      lsm6ds1.setAccelDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU1_accel_gyro_rate);
      lsm6ds1.setGyroDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU1_accel_gyro_rate);
      lsm6ds1.setAccelRange((lsm6ds_accel_range_t)configData.configDataIMUJOTISK.IMU1_accel_range);
      lsm6ds1.setGyroRange((lsm6ds_gyro_range_t)configData.configDataIMUJOTISK.IMU1_gyro_range);
      lis3mdl1.setDataRate((lis3mdl_dataRate_t)configData.configDataIMUJOTISK.IMU1_mag_freq);
      lis3mdl1.setRange((lis3mdl_range_t)configData.configDataIMUJOTISK.IMU1_mag_range);

      // Configure IMU2
      lsm6ds2.setAccelDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU2_accel_gyro_freq);
      lsm6ds2.setGyroDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU2_accel_gyro_freq);
      lsm6ds2.setAccelRange((lsm6ds_accel_range_t)configData.configDataIMUJOTISK.IMU2_accel_range);
      lsm6ds2.setGyroRange((lsm6ds_gyro_range_t)configData.configDataIMUJOTISK.IMU2_gyro_range);
      lis3mdl2.setDataRate((lis3mdl_dataRate_t)configData.configDataIMUJOTISK.IMU2_mag_freq);
      lis3mdl2.setRange((lis3mdl_range_t)configData.configDataIMUJOTISK.IMU2_mag_range);

      Serial.println("Settings restored successfully");
      return true;
    }
    Serial.println("No valid settings found");
    return false;
  }
  Serial.println("Failed to initialize EEPROM");
  return false;
}


/* 
brief :
Sets the default settings for the IMU and Joystick configurations. 

*/
void setDefaultSettings()
{

  IMU_config_data_anJoystick_packed configData;

  configData.configDataIMUJOTISK.IMU1_accel_gyro_rate = LSM6DS_RATE_12_5_HZ;
  configData.configDataIMUJOTISK.IMU1_mag_freq = LIS3MDL_DATARATE_155_HZ;
  configData.configDataIMUJOTISK.IMU1_accel_range = LSM6DS_ACCEL_RANGE_2_G;
  configData.configDataIMUJOTISK.IMU1_gyro_range = LSM6DS_GYRO_RANGE_250_DPS;
  configData.configDataIMUJOTISK.IMU1_mag_range = LIS3MDL_RANGE_4_GAUSS;

  configData.configDataIMUJOTISK.IMU2_accel_gyro_freq = LSM6DS_RATE_12_5_HZ;
  configData.configDataIMUJOTISK.IMU2_mag_freq = LIS3MDL_DATARATE_155_HZ;
  configData.configDataIMUJOTISK.IMU2_accel_range = LSM6DS_ACCEL_RANGE_2_G;
  configData.configDataIMUJOTISK.IMU2_gyro_range = LSM6DS_GYRO_RANGE_250_DPS;
  configData.configDataIMUJOTISK.IMU2_mag_range = LIS3MDL_RANGE_4_GAUSS;

  configData.configDataIMUJOTISK.Joystick_flex_sensor_rate = 100;

  uint16_t validationFlag = SETTINGS_VAID_FLAG;
  EEPROM.put(SETTING_ADDRESS, validationFlag);
  EEPROM.put(SETTING_ADDRESS + sizeof(validationFlag), configData);
  EEPROM.commit();

  Serial.println("Default settings saved");
}

void setTupBLEGamepad() {


  
}



void setup()
{
  Serial.begin(115200);
  systemInit();
  if (!Restoresettings())
  {
    Serial.println("Setting default configuration");
    setDefaultSettings();
  }

  if (!initFlueGaue())
  {
    Serial.println("failed");
  }

  if (!initIMU())
  {
    Serial.println("Failed to initialize IMU chips");

  }
  else
  {
    setupIMUdataRate();

  }

  setupIMUInterrupts(); 

  if(LoadCallibration()) {
    Serial.println("Callibration loaded successfully");
  } else {
    Serial.println("Failed to load callibration");
  }

  setTupBLEGamepad() ; 


}

void loop()
{

  delay(1000);
  Serial.println("Hello World");
}