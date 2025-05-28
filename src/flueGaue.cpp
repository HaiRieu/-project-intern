#include "flueGaue.h" 
#include <Wire.h> 

#define FUEL_GAUGE_I2C_SDA_PIN  34
#define FUEL_GAUGE_SCL 33 



Overall_status_data_packed overallStatusDatapPacked; 

/*
brief :
   Initialize the fuel gauge and set the activity threshold. 
*/

bool initFlueGaue()
{
 
  Wire.begin(FUEL_GAUGE_I2C_SDA_PIN, FUEL_GAUGE_SCL);

  if (!fuelGauge.begin(&Wire))
  {
    Serial.println("Failed to find MAX17048 chip");
    overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::FAILED ; 
    overallStatusDatapPacked.overallStatusData.status_code = statuscode::GENERAL_ERROR ; 
    feulGaue = false;
    return false;
  }
  {
    Serial.println("Fuel Gauge not found");
    overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::NOT_DETECT ; 
    feulGaue = false;
  }

  Serial.println("find fuel gauge ");
  feulGaue = true;

  fuelGauge.setActivityThreshold(10);
  overallStatusDatapPacked.overallStatusData.Fuelgause_status = statuscode_sensor::RUNNING ;
  return true;
}
