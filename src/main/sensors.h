/*
    sensors.h - Library for interfacing with the sensors on the parachute.
    Name: Nischay Joshi
    Date: 07-12-23
    Description: Header file for sensors.c
    Contains function and class prototypes for GPS, IMU and Barometer. 
*/

#ifndef SENSORS_H
#define SENSORS_H

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_BMP280.h>
#include "gpsLowLevel.h"
#include "vector_and_quaternion.h"
#include <EEPROM.h>


#define SERIAL_PORT Serial
#define WIRE_PORT Wire

#define I2C_SDA 21
#define I2C_SCL 22

// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
// #define GRAVITY 9.809f

// BMP 280 CONFIG
#define BMP_ADDRESS 0x77
#define BMP_CHIP_ID 0x60
#define BMP_SDO 15
#define SEALEVELPRESSURE_HPA (1026.9)

// GPS CONFIG
#define GPSSERIAL Serial2
#define GPSBAUDRATE 115200
#define GPS_RX 4
#define GPS_TX 2
#define GPS_RATE 10

// BATTERY MEASUREMENT
#define BATTERY_PIN 14

// YAW OFFSET
#define YAW_OFFSET 180.0

namespace Sensors{

// Define a storage struct for the biases. Include a non-zero header and a simple checksum
typedef struct {
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
} biasStore;

typedef struct{
    Vector RawAccel;
    Quaternion Orientation;
    Vector LinearAccel;
    Vector LinearAccelOffset;
    Vector EulerAngles;
    biasStore IMUDmpBias;
    bool imuBiasFoundinEEPROM;
    float IMU_HeadOffset;
} imuData_t;

typedef struct{
    float Temperature;
    float Pressure;
    float Altitude;
    float AltitudeOffset;
} barometerData_t;

typedef struct{
    uint16_t ReadValue;
    float ReadVoltage;
    float LipoVoltage;
} batteryData_t;

typedef struct{
    imuData_t imuData;
    barometerData_t barometerData;
    gpsData_t gpsData;
    batteryData_t batteryData;
} sensorData_t;

typedef enum{
        SENSORS_OK,
        SENSORS_FAIL
} SENSORS_Status_t;

class sensors{
    public:
    sensors(): gps(&GPSSERIAL, GPSBAUDRATE, GPS_RATE, GPS_RX, GPS_TX){}; 
    SENSORS_Status_t init();
    SENSORS_Status_t readData_noGPS(sensorData_t * sensorData_Out);
    SENSORS_Status_t readData_GPS(sensorData_t * sensorData_Out);
    sensorData_t sensorData;
    SENSORS_Status_t CalibrateIMULinearAcceleration();
    SENSORS_Status_t CalibrateBarometerAltitude();
    SENSORS_Status_t UpdateBatteryData(sensorData_t * sensorData_Out);
    void PrintGPSData();
    void resetGPSReference();

    private:
    //ICM_20948 Object
    ICM_20948_I2C imu;
    //BMP280 Object
    Adafruit_BMP280 bmp;
    //GPS Object
    gpsLowLevel gps;

    //Sensor Data
    SENSORS_Status_t initSensorDataStruct();
    
    //IMU Functions
    SENSORS_Status_t initIMU();
    SENSORS_Status_t readIMUData();
    void updateBiasStoreSum(biasStore *store);
    bool isBiasStoreValid(biasStore *store);
    void printBiases(biasStore *store);

    //Barometer Functions
    SENSORS_Status_t initBarometer();
    SENSORS_Status_t readBarometerData();

    //GPS Functions
    SENSORS_Status_t initGPS();
    SENSORS_Status_t readGPSData();

    //Battery Functions
    SENSORS_Status_t initBattery();
    SENSORS_Status_t readBatteryData();
};

}// namespace Sensors
#endif // SENSORS_H