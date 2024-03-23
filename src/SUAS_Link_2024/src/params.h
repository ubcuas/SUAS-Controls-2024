// Define parameters (constants, ESP pins, etc.)

#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>

extern HardwareSerial PiSerial;
extern HardwareSerial CubeSerial;

#define SERVO_FRONT_R 32
#define SERVO_BACK_R 27
#define SERVO_FRONT_L 33  
#define SERVO_BACK_L 13

#define LED_RED 23
#define LED_BLUE 21

#define M_PER_LAT_DEG 111320.0 // 111320 meters per degree of latitude (short distance approx)

#define AIRCRAFT_SPEED 7.0 // m/s
#define DRIFT_FACTOR 0.1 // TODO: find this from testing, it's arbitrary rn!!
#define RELEASE_DELAY 1.0 // s

#define WINDOW_SIZE 10

#endif