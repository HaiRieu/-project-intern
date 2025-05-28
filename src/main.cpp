#include <Arduino.h>

#include "flueGaue.h"
#include "app.h"
#include "imu.h"




void systemInit() {
  pinMode(VSVY_EN_PIN, OUTPUT);
  digitalWrite(VSVY_EN_PIN, LOW);
  Wire.begin(33, 34);  // SCL = 33, SDA = 34
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
