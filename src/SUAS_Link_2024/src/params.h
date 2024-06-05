// Define parameters (constants, ESP pins, etc.)

#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>

#define CUBE_BAUD_RATE 57600
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
#define DRIFT_FACTOR 0.0 // TODO: find this from testing, it's arbitrary rn!!
#define RELEASE_DELAY 0.0 // s // TODO!!!
#define RELEASE_MARGIN 6.0 // Tolerance margin from desired drop point that we will trigger release

#define WINDOW_SIZE 10

#define FAILSAFE_MODE true

// MAC Address of responder - edit as required
static const uint8_t ADDRESS_1[] = {0xD8, 0xBC, 0x38, 0xE4, 0x9E, 0x5C};
// static const uint8_t ADDRESS_2[] = {0x, 0x, 0x, 0x, 0x, 0x};
// static const uint8_t ADDRESS_3[] = {0x, 0x, 0x, 0x, 0x, 0x};
// static const uint8_t ADDRESS_4[] = {0x, 0x, 0x, 0x, 0x, 0x};
// static const uint8_t ADDRESS_5[] = {0x, 0x, 0x, 0x, 0x, 0x};

#endif