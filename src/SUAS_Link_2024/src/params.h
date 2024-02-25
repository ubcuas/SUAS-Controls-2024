// Define parameters (constants, ESP pins, etc.)

#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H

#include <Arduino.h>

extern HardwareSerial PiSerial;
extern HardwareSerial CubeSerial;

#define AIRCRAFT_SPEED 7.0 // m/s
#define DRIFT_FACTOR 0.1 // TODO: find this from testing, it's arbitrary rn!!
#define RELEASE_DELAY 1.0 // s


#endif