#include "appsetup.h"

uint32_t timestamp;
bool IMUsAvailable = false;
Adafruit_NXPSensorFusion fillsion1, fillsion2;

IMU1_data_Raw_packed imu1DataRawPacked;
IMU2_data_Raw_packed imu2DataRawPacked;

/*
brief Initializes the IMU sensors and sets their data rates.
@param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
*/

bool initIMU(SensorGroupIMU &sensorGroup)
{
  Overall_status_data_packed overallStatusDatapPacked;

  if (!sensorGroup.lsm6ds1->begin_I2C(0x6A))
  {
    Serial.println("Failed to find LSM6DS1 chip");
    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;

    overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::FAILED;
    return false;
  }
  if (!sensorGroup.lsm6ds2->begin_I2C(0x6B))
  {
    Serial.println("Failed to find LSM6DS2 chip");
    overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::FAILED;
    return false;
  }
  if (!sensorGroup.lis3mdl1->begin_I2C(0x1E))
  {
    Serial.println("Failed to find LIS3MDL chip 1");
    return false;
  }
  if (!sensorGroup.lis3mdl1->begin_I2C(0x1C))
  {
    Serial.println("Failed to find LIS3MDL chip 2");
    return false;
  }

  fillsion1.begin(FILTER_UPDATE_RATE_HZ);
  fillsion2.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  overallStatusDatapPacked.overallStatusData.Imu1_status = statuscode_sensor::RUNNING;
  overallStatusDatapPacked.overallStatusData.Imu2_status = statuscode_sensor::RUNNING;
  IMUsAvailable = true;
  return true;
}


/*
brief Sets up the IMU data rate and range for the sensors.
@param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors

*/

void setupIMUDataRate(SensorGroupIMU &sensorGroup)
{
  sensorGroup.lsm6ds1->setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sensorGroup.lsm6ds1->setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  sensorGroup.lsm6ds1->setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sensorGroup.lsm6ds1->setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  sensorGroup.lis3mdl1->setDataRate(LIS3MDL_DATARATE_155_HZ);
  sensorGroup.lis3mdl1->setRange(LIS3MDL_RANGE_4_GAUSS);

  sensorGroup.lsm6ds2->setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sensorGroup.lsm6ds2->setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  sensorGroup.lsm6ds2->setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sensorGroup.lsm6ds2->setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  sensorGroup.lis3mdl2->setDataRate(LIS3MDL_DATARATE_155_HZ);
  sensorGroup.lis3mdl2->setRange(LIS3MDL_RANGE_4_GAUSS);
}


/*

brief Reads data from the IMU sensors and updates the imu1DataRawPacked and imu2DataRawPacked structures.
@param sensorGroup Reference to the SensorGroupIMU object containing IMU sensors
This function reads acceleration, gyroscope, and magnetometer data from two IMU sensors (LSM6DS1 and LSM6DS2) and stores the raw data in packed structures (imu1DataRawPacked and imu2DataRawPacked). It also checks the time interval to ensure data is read at a specified update rate.
*/

void readDataIMU(SensorGroupIMU &sensorGroup)
{

  if (IMUsAvailable)
  {

    sensors_event_t accel1, gyro1, mag1;
    sensors_event_t accel2, gyro2, mag2;

    sensorGroup.lsm6ds1->getEvent(&accel1, &gyro1, NULL);
    sensorGroup.lsm6ds2->getEvent(&accel2, &gyro2, NULL);
    sensorGroup.lis3mdl1->getEvent(&mag1);
    sensorGroup.lis3mdl2->getEvent(&mag2);

    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
    {
      return;
    }

    imu1DataRawPacked.data_Imu1.accelX = accel1.acceleration.x;
    imu1DataRawPacked.data_Imu1.accelY = accel1.acceleration.y;
    imu1DataRawPacked.data_Imu1.accelZ = accel1.acceleration.z;

    imu1DataRawPacked.data_Imu1.GyroX = gyro1.gyro.x;
    imu1DataRawPacked.data_Imu1.GyroY = gyro1.gyro.y;
    imu1DataRawPacked.data_Imu1.GyroZ = gyro1.gyro.z;

    imu1DataRawPacked.data_Imu1.MagX = mag1.magnetic.x;
    imu1DataRawPacked.data_Imu1.MagY = mag1.magnetic.y;
    imu1DataRawPacked.data_Imu1.MagZ = mag1.magnetic.z;

    imu2DataRawPacked.data_Imu2.accelX = accel2.acceleration.x;
    imu2DataRawPacked.data_Imu2.accelY = accel2.acceleration.y;
    imu2DataRawPacked.data_Imu2.accelZ = accel2.acceleration.z;

    imu2DataRawPacked.data_Imu2.GyroX = gyro2.gyro.x;
    imu2DataRawPacked.data_Imu2.GyroY = gyro2.gyro.y;
    imu2DataRawPacked.data_Imu2.GyroZ = gyro2.gyro.z;

    imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.x;
    imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.y;
    imu2DataRawPacked.data_Imu2.MagX = mag2.magnetic.z;


  }
}
