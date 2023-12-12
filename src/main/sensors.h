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


#define SERIAL_PORT Serial
#define WIRE_PORT Wire

#define I2C_SDA 21
#define I2C_SCL 22

// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#define GRAVITY 9.809f

// BMP 280 CONFIG
#define BMP_ADDRESS 0x76
#define BMP_CHIP_ID 0x58
#define SEALEVELPRESSURE_HPA (1026.9)

// GPS CONFIG
#define GPSSERIAL Serial2
#define GPSBAUDRATE 115200
#define GPS_RX 2
#define GPS_TX 4
#define GPS_RATE 5



namespace Sensors{

typedef struct{
    Vector RawAccel;
    Quaternion Orientation;
    Vector LinearAccel;
    Vector LinearAccelOffset;
} imuData_t;

typedef struct{
    float Temperature;
    float Pressure;
    float Altitude;
    float AltitudeOffset;
} barometerData_t;


typedef struct{
    imuData_t imuData;
    barometerData_t barometerData;
    gpsData_t gpsData;
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
    void PrintGPSData();

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
    

    //Barometer Functions
    SENSORS_Status_t initBarometer();
    SENSORS_Status_t readBarometerData();
    

    //GPS Functions
    SENSORS_Status_t initGPS();
    SENSORS_Status_t readGPSData();
};

}// namespace Sensors
#endif // SENSORS_H