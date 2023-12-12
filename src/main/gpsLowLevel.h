/*
    gpsLowLevel.h - Wrapper Library for interfacing with the GPS.
    Name: Nischay Joshi
    Date: 07-12-23
*/
#ifndef GPS_LOW_LEVEL_H
#define GPS_LOW_LEVEL_H

#include "Arduino.h"
#include <TinyGPSPlus.h>


//if you add more data to the gpsData_t struct, make sure to update the functions in gpsLowLevel.cpp
typedef struct{
    float Latitude;
    float Longitude;
    float Altitude;
    bool lock;
    int satellites;
} gpsData_t;

class gpsLowLevel {
public:
    gpsLowLevel(HardwareSerial *gpsSerial = &Serial2, uint32_t GPSBaudRate = 115200, uint8_t GPSRate = 5, uint8_t RXPin = 2, uint8_t TXPin = 4);
    
    enum GPS_Status {
        GPS_OK,
        GPS_FAIL
    };
    GPS_Status begin();
    GPS_Status update();
    GPS_Status fetchAllData(gpsData_t * gpsData_Out);

private:
    HardwareSerial *gpsSerial;
    uint8_t RXPin;
    uint8_t TXPin;
    uint32_t GPSBaudRate;
    uint8_t GPSRate;
    TinyGPSPlus gps;
    gpsData_t gpsData;

    // Additional private member variables and methods as needed
};

#endif // GPS_LOW_LEVEL_H