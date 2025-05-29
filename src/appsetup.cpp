#include "appsetup.h" 


bool restoreSettings(SensorGroupIMU &sensorGroup);
void setDefaultSettings();
bool initIMU(SensorGroupIMU &sensorGroup);
void setupIMUDataRate(SensorGroupIMU &sensorGroup);
void setupIMUInterrupts(SensorGroupIMU &sensorGroup);
void IRAM_ATTR imu1InterruptHandler();
bool loadCalibration();
void setupBLEGamepad(ble &bleGamepad); ; 
bool initFuelGauge() ;
void ledRGB() ; 
void systemInit() ;
void updateOverallStatusData() ; 


SensorGroupIMU sensorGroup;
ble bleGamepad; 

/*
brief Setup the application
This function initializes the application by setting up the serial communication, restoring settings, initializing the fuel gauge, IMU sensors, loading calibration data, and setting up the BLE gamepad.
*/

void setupApp() 
{
  Serial.begin(115200);
  Serial.println("Setting up application...");

  systemInit() ; 

  if (!restoreSettings(sensorGroup)) {
    Serial.println("No valid settings found, setting default configuration");
    setDefaultSettings();
  }


  if (!initFuelGauge()) {
    Serial.println("Fuel gauge initialization failed");
  }


  if (!initIMU(sensorGroup)) {
    Serial.println("IMU initialization failed");
  } else {
    setupIMUDataRate(sensorGroup);
    setupIMUInterrupts(sensorGroup);
  }

  if (loadCalibration()) {
    Serial.println("Calibration loaded successfully");
  } else {
    Serial.println("Failed to load calibration");
  }
  setupBLEGamepad(bleGamepad);
  ledRGB() ; 
  updateOverallStatusData();
}

