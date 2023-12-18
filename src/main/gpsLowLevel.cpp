/*
    gpsLowLevel.cpp - Library for interfacing with the GPS module.
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include "gpsLowLevel.h"

gpsLowLevel::gpsLowLevel(HardwareSerial *gpsSerial, uint32_t GPSBaudRate, uint8_t GPSRate, uint8_t RXPin, uint8_t TXPin)
    : gpsSerial(gpsSerial), GPSBaudRate(GPSBaudRate), GPSRate(GPSRate), RXPin(RXPin), TXPin(TXPin){}

gpsLowLevel::GPS_Status gpsLowLevel::begin(){
    gpsSerial->begin(GPSBaudRate, SERIAL_8N1, RXPin, TXPin);

    //intialize the Gps data struct
    gpsData.Latitude = 0;
    gpsData.Longitude = 0;
    gpsData.Altitude = 0;
    gpsData.lock = false;
    gpsData.satellites = 0;
    gpsData.refLatitude = 0;
    gpsData.refLongitude = 0;
    gpsData.velocity = 0;
    gpsData.Xpos = 0;
    gpsData.Ypos = 0;

    return GPS_OK;
}

gpsLowLevel::GPS_Status gpsLowLevel::update(){
    while (gpsSerial->available() > 0)
        if (gps.encode(gpsSerial->read()))
            return GPS_OK;
    return GPS_FAIL;
}

gpsLowLevel::GPS_Status gpsLowLevel::fetchAllData(gpsData_t * gpsData_Out){
    //update the local gpsData struct
    if(gps.location.isValid()){
        gpsData.Altitude = gps.altitude.meters();
        gpsData.Latitude = gps.location.lat();
        gpsData.Longitude = gps.location.lng();
        gpsData.lock = true;
        gpsData.satellites = gps.satellites.value();
        //get velocity in m/s
        gpsData.velocity = gps.speed.mps();
        // //get the reference position
        // gpsData.refLatitude = gps.location.lat();
        // gpsData.refLongitude = gps.location.lng();
        //get the X and Y position
        gpsData.Xpos = distanceBetween(gpsData.refLatitude, gpsData.refLongitude, gpsData.refLatitude, gpsData.Longitude);
        gpsData.Ypos = distanceBetween(gpsData.refLatitude, gpsData.refLongitude, gpsData.Latitude, gpsData.refLongitude);
    }
    else{
        //keep the old data just set the lock to false
        gpsData.lock = false;
    }

    //copy the local gpsData struct to the output struct
    *gpsData_Out = gpsData;

    return GPS_OK;
}

float gpsLowLevel::distanceBetween(float lat1, float long1, float lat2, float long2){
    float dist = TinyGPSPlus::distanceBetween(lat1, long1, lat2, long2);
    if(long2 < long1)
        dist = -1.0*dist;
    else if(lat2 < lat1)
        dist = -1.0*dist;
    return dist;
}