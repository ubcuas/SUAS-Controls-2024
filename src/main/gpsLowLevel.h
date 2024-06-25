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
    double Latitude;
    double Longitude;
    float Altitude;
    bool lock;
    int satellites;
    double refLatitude;
    double refLongitude;
    float velocity;
    double Xpos;
    double Ypos;
} gpsData_t;

class gpsLowLevel {
public:
    gpsLowLevel(HardwareSerial *gpsSerial = &Serial2, uint32_t GPSBaudRate = 115200, uint8_t GPSRate = 5, uint8_t RXPin = 4, uint8_t TXPin = 2);
    
    enum GPS_Status {
        GPS_OK,
        GPS_FAIL
    };
    GPS_Status begin();
    GPS_Status update();
    GPS_Status fetchAllData(gpsData_t * gpsData_Out);
    GPS_Status resetReference();
    double distBetween(double lat1, double long1, double lat2, double long2);

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