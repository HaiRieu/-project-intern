#include "flueGaue.h" 
#include <Wire.h> 

#define FUEL_GAUGE_I2C_SDA_PIN 34
#define FUEL_GAUGE_I2C_SCL_PIN 33

Adafruit_MAX17048 fuelGauge;
 Overall_status_data_packed overallStatusDatapPacked;

bool initFuelGauge() {

  Wire.begin(FUEL_GAUGE_I2C_SDA_PIN, FUEL_GAUGE_I2C_SCL_PIN);
 
  statuscode statusCode;

  if (!fuelGauge.begin(&Wire)) {
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