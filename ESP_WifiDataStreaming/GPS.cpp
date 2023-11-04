#include <TinyGPSPlus.h>
#include "GPS.h"
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and aSerial2umes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
// static const int RXPin = 4, TXPin = 3;
const uint32_t GPSBaud = 9600;
int lastUpdateMillis = -1;
String lastInfo = "NOTHING";


// The TinyGPSPlus object
TinyGPSPlus gps;

void init_gps()
{
  Serial2.begin(GPSBaud);
}

String getInfo() {
  return lastInfo;
}

// Format: controller millis(), latitude, longidude, altitude (feet), GPS date (MM/DD/YYYY), GPS time(hh:mm:ss.00), number of satellites
String displayInfo()
{
  String s = String(millis()) + ",";

  if (gps.location.isValid())
  {
    s += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  }
  else
  {
    s += "INVALID,INVALID";
  }

  s += ",";
  if (gps.altitude.isValid())
  {
    s += String(gps.altitude.feet(), 1);
  }
  else {
    s += "INVALID";
  }

  s += ",";
  if (gps.date.isValid())
  {
    s += String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
  }
  else
  {
    s += "INVALID";
  }

  s += ",";
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) s += "0";
    s += String(gps.time.hour()) + ":";
    if (gps.time.minute() < 10) s += "0";
    s += String(gps.time.minute()) + ":";
    if (gps.time.second() < 10) s += "0";
    s += String(gps.time.second()) + ".";
    if (gps.time.centisecond() < 10) s += "0";
    s += String(gps.time.centisecond());
  }
  else
  {
    s += "INVALID";
  }

  s += ",";
  if (gps.satellites.isValid())
  {
    s += String(gps.satellites.value());
  }
  else {
    s += "NO SATELLITES";
  }

  return s;
}

// return: whether new GPS info is available or not
bool updateGPS() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      lastUpdateMillis = millis();
      lastInfo = displayInfo();
      return true;
    }
  }
  return false;
}