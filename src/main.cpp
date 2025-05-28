#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>
#include <Adafruit_MAX1704X.h>
#include <BleGamepad.h>
#include "flueGaue.h"
#include "app.h"
#include "imu.h"


#define VSVY_EN_PIN 18


void systemInit() {
  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, LOW);
  Wire.begin(33, 34);  // SCL = 33, SDA = 34
}

void setupBLEGamepad() {
  // Placeholder for BLE gamepad initialization
}

void setup() {
  Serial.begin(115200);
  systemInit();

  if (!restoreSettings()) {
    Serial.println("Setting default configuration");
    setDefaultSettings();
  }

  if (!initFuelGauge()) {
    Serial.println("Fuel gauge init failed");
  }

  if (!initIMU()) {
    Serial.println("IMU initialization failed");
  } else {
    setupIMUDataRate();
    setupIMUInterrupts();
  }

  if (loadCalibration()) {
    Serial.println("Calibration loaded successfully");
  } else {
    Serial.println("Failed to load calibration");
  }

  setupBLEGamepad();
}

void loop() {
  delay(1000);
  Serial.println("Hello World");
}
