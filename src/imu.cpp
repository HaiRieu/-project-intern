#include "imu.h"
#include <EEPROM.h>

Adafruit_LSM6DS3TRC lsm6ds1;
Adafruit_LSM6DS3TRC lsm6ds2;
Adafruit_LIS3MDL lis3mdl1;
Adafruit_LIS3MDL lis3mdl2;

volatile bool IMUsAvailable = false;
volatile bool imuDataReady = false;
bool calibrationLoaded = false;


IMU1_euler_calib_status_packed imu1EulerCalibration;
IMU2_euler_calib_status_packed imu2EulerCalibration;

#define CALIBRATION_ADDRESS 0x100
#define SETTING_ADDRESS 0x00
#define SETTINGS_VALID_FLAG 0xAA55

bool restoreSettings() {
  if (EEPROM.begin(512)) {
    IMU_config_data_anJoystick_packed configData;
    uint16_t validationFlag;

    EEPROM.get(SETTING_ADDRESS, validationFlag);
    if (validationFlag == SETTINGS_VALID_FLAG) {
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

void setDefaultSettings() {
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

  uint16_t validationFlag = SETTINGS_VALID_FLAG;
  EEPROM.put(SETTING_ADDRESS, validationFlag);
  EEPROM.put(SETTING_ADDRESS + sizeof(validationFlag), configData);
  EEPROM.commit();

  Serial.println("Default settings saved");
}

bool initIMU() {
  Overall_status_data_packed overallStatusDatapPacked;
  
  if (!lsm6ds1.begin_I2C(0x6A)) {
    Serial.println("Failed to find LSM6DS1 chip");
     overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    return false;
  }
  if (!lsm6ds2.begin_I2C(0x6B)) {
    Serial.println("Failed to find LSM6DS2 chip");
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
    return false;
  }
  if (!lis3mdl1.begin_I2C(0x1E)) {
    Serial.println("Failed to find LIS3MDL chip 1");
    return false;
  }
  if (!lis3mdl2.begin_I2C(0x1C)) {
    Serial.println("Failed to find LIS3MDL chip 2");
    return false;
  }

  overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
  overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
  IMUsAvailable = true;
  return true;
}

void setupIMUDataRate() {
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

void setupIMUInterrupts() {
  pinMode(IMU1_INT_PIN, INPUT);
  pinMode(IMU2_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1InterruptHandler, RISING);
  lsm6ds1.configInt1(false, false, true);
  lsm6ds2.configInt1(false, false, true);
  Serial.println("IMU interrupts configured");
}

void IRAM_ATTR imu1InterruptHandler() {
  imuDataReady = true;
}

bool loadCalibration() {
  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialize EEPROM");
    return false;
  }

  EEPROM.get(CALIBRATION_ADDRESS, imu1EulerCalibration);
  EEPROM.get(CALIBRATION_ADDRESS + sizeof(IMU1_euler_calib_status_packed), imu2EulerCalibration);

  if (imu1EulerCalibration.eulerCalibStatus.calibation != 0 &&
      imu2EulerCalibration.eulerCalibStatus.calibation != 0) {
    calibrationLoaded = true;
    return true;
  }

  calibrationLoaded = false;
  return false;
}
