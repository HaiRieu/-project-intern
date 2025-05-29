#include <app.h>

IMU1_euler_calib_status_packed imu1EulerCalibration;
IMU2_euler_calib_status_packed imu2EulerCalibration;
calibratee calibrationData; 
BleGamepad bleGamepad;
BleGamepadConfiguration bleGamepadConfig;

Adafruit_MAX17048 fuelGauge;
Overall_status_data_packed overallStatusDatapPacked;

bool imuDataReady = false;
bool calibrationLoaded = false;

void systemInit()
{
  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, LOW);
  Wire.begin(33, 34);
}
/* 
brief Restore settings from EEPROM
 * 
 * @param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
 * @return true if settings were restored successfully, false otherwise
 * 
 * This function reads the configuration data for the IMUs from EEPROM and applies it to the sensors.
 * It checks for a valid settings flag before restoring the settings.
*/

bool restoreSettings(SensorGroupIMU &sensorGroup)
{
  if (EEPROM.begin(512))
  {
    IMU_config_data_anJoystick_packed configData;
    uint16_t validationFlag;

    EEPROM.get(SETTING_ADDRESS, validationFlag);
    if (validationFlag == SETTINGS_VALID_FLAG)
    {
      EEPROM.get(SETTING_ADDRESS + sizeof(validationFlag), configData);

      // Configure IMU1
      sensorGroup.lsm6ds1->setAccelDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU1_accel_gyro_rate);
      sensorGroup.lsm6ds1->setGyroDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU1_accel_gyro_rate);
      sensorGroup.lsm6ds1->setAccelRange((lsm6ds_accel_range_t)configData.configDataIMUJOTISK.IMU1_accel_range);
      sensorGroup.lsm6ds1->setGyroRange((lsm6ds_gyro_range_t)configData.configDataIMUJOTISK.IMU1_gyro_range);
      sensorGroup.lis3mdl1->setDataRate((lis3mdl_dataRate_t)configData.configDataIMUJOTISK.IMU1_mag_freq);
      sensorGroup.lis3mdl1->setRange((lis3mdl_range_t)configData.configDataIMUJOTISK.IMU1_mag_range);

      // Configure IMU2
      sensorGroup.lsm6ds2->setAccelDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU2_accel_gyro_freq);
      sensorGroup.lsm6ds2->setGyroDataRate((lsm6ds_data_rate_t)configData.configDataIMUJOTISK.IMU2_accel_gyro_freq);
      sensorGroup.lsm6ds2->setAccelRange((lsm6ds_accel_range_t)configData.configDataIMUJOTISK.IMU2_accel_range);
      sensorGroup.lsm6ds2->setGyroRange((lsm6ds_gyro_range_t)configData.configDataIMUJOTISK.IMU2_gyro_range);
      sensorGroup.lis3mdl2->setDataRate((lis3mdl_dataRate_t)configData.configDataIMUJOTISK.IMU2_mag_freq);
      sensorGroup.lis3mdl2->setRange((lis3mdl_range_t)configData.configDataIMUJOTISK.IMU2_mag_range);

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
brief Set default settings for IMU and Joystick
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

  uint16_t validationFlag = SETTINGS_VALID_FLAG;
  EEPROM.put(SETTING_ADDRESS, validationFlag);
  EEPROM.put(SETTING_ADDRESS + sizeof(validationFlag), configData);
  EEPROM.commit();

  Serial.println("Default settings saved");
}

/*
brief Interrupt handler for IMU
*/
void IRAM_ATTR imu1InterruptHandler()
{
  imuDataReady = true;
}


/*
brief Setup IMU interrupts
 * @param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
*/
void setupIMUInterrupts(SensorGroupIMU &sensorGroup)
{
  pinMode(IMU1_INT_PIN, INPUT);
  pinMode(IMU2_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1InterruptHandler, RISING);
  sensorGroup.lsm6ds1->configInt1(false, false, true);
  sensorGroup.lsm6ds1->configInt1(false, false, true);
  Serial.println("IMU interrupts configured");
}

/*
brief Load calibration data from EEPROM

 * @return true if calibration data was loaded successfully, false otherwise

*/

bool loadCalibration()
{
  if (!EEPROM.begin(512))
  {
    Serial.println("Failed to initialize EEPROM");
    return false;
  }

  EEPROM.get(CALIBRATION_ADDRESS, imu1EulerCalibration);
  EEPROM.get(CALIBRATION_ADDRESS + sizeof(IMU1_euler_calib_status_packed), imu2EulerCalibration);

  if (imu1EulerCalibration.eulerCalibStatus.calibation != 0 &&
      imu2EulerCalibration.eulerCalibStatus.calibation != 0)
  {
    calibrationLoaded = true;
       calibrationData.calibationIMU1  =  imu1EulerCalibration.eulerCalibStatus.calibation ;
       calibrationData.calibationIMU2 = imu2EulerCalibration.eulerCalibStatus.calibation ; 

       imu1EulerCalibration.eulerCalibStatus.calibation =  calibrationData.calibationIMU1 ;
       imu2EulerCalibration.eulerCalibStatus.calibation  = calibrationData.calibationIMU2 ; 

    return true;
  }

  calibrationLoaded = false;
       calibrationData.calibationIMU1  =  imu1EulerCalibration.eulerCalibStatus.calibation ;
       calibrationData.calibationIMU2 = imu2EulerCalibration.eulerCalibStatus.calibation ; 

       imu1EulerCalibration.eulerCalibStatus.calibation =  calibrationData.calibationIMU1 ;
       imu2EulerCalibration.eulerCalibStatus.calibation  = calibrationData.calibationIMU2 ; 
  return false;
}


/*
brief Setup BLE Gamepad
 
 * This function initializes the BLE Gamepad with the specified configuration.

*/
void setupBLEGamepad()
{

  Serial.println("Starting BLE work!");
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
  bleGamepadConfig.setVid(0xe502);
  bleGamepadConfig.setPid(0xabcd);

  bleGamepadConfig.setModelNumber(const_cast<char *>("ESP32-G1"));
  bleGamepadConfig.setSoftwareRevision(const_cast<char *>("v1.0.0"));
  bleGamepadConfig.setSerialNumber(const_cast<char *>("SN001"));
  bleGamepadConfig.setFirmwareRevision(const_cast<char *>("FW1.0"));
  bleGamepadConfig.setHardwareRevision(const_cast<char *>("HW1.0"));

  bleGamepadConfig.setAxesMin(0x0000);
  bleGamepadConfig.setAxesMax(0x7FFF);

  bleGamepad.begin(&bleGamepadConfig);
}


/*
brief Initialize the fuel gauge

 * This function initializes the MAX17048 fuel gauge using I2C communication.
 * It sets the activity threshold and checks if the device is ready.
 * 
 * @return true if the fuel gauge is initialized successfully, false otherwise
*/
bool initFuelGauge()
{

  Wire.begin(FUEL_GAUGE_I2C_SDA_PIN, FUEL_GAUGE_I2C_SCL_PIN);

  if (!fuelGauge.begin(&Wire))
  {
    Serial.println("Failed to find MAX17048 chip");
    overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::FAILED;
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR;
    return false;
  }
  Serial.println("Fuel gauge found");
  fuelGauge.setActivityThreshold(10);
  overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::RUNNING;
  return true;
}

/*
brief Update the overall status data

 * This function updates the overall status data structure with the current status of the fuel gauge and IMU sensors.
 * It sets the status code and sensor statuses based on their readiness.
*/

void updateOverallStatusData()
{
  overallStatusDatapPacked.overallStatusData.status_code = statuscode::NO_ERROR;
  overallStatusDatapPacked.overallStatusData.Fuelgause_status = fuelGauge.isDeviceReady() ? statuscode_sensor::RUNNING : statuscode_sensor::FAILED;

  if (imuDataReady)
  {
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
  }
  else
  {
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
  }
}
