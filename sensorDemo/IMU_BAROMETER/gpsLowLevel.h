/*
    gpsLowLevel.h - Wrapper Library for interfacing with the GPS.
    Name: Nischay Joshi
    Date: 07-12-23
*/
#ifndef GPS_LOW_LEVEL_H
#define GPS_LOW_LEVEL_H

#include "Arduino.h"
#include <TinyGPSPlus.h>

// GPS CONFIG
#define GPSSERIAL Serial2
#define GPSBAUDRATE 115200
#define GPS_RX 2
#define GPS_TX 4

class gpsLowLevel{
    public:
    HardwareSerial *gpsSerial = &GPSSERIAL;
    TinyGPSPlus gps;
    typedef enum{
        GPS_OK,
        GPS_FAIL
        } GPS_Status;

    private:
    uint8_t RX_PIN = GPS_RX;
    uint8_t TX_PIN = GPS_TX;
    uint32_t GPS_Baud_Rate = GPSBAUDRATE;
    uint8_t GPS_Rate = 5;
};

#endif // GPS_LOW_LEVEL_H