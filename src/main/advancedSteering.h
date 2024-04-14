/*
 * Autonomous Parachute Steering Functon Header
 * Keeping track of the angle of a continous rotation servo
 * Author: Nischay Joshi
 * Created: April 13th, 2024
 */

#ifndef ADVANCEDSTEERING_H
#define ADVANCEDSTEERING_H

#include <ESP32Servo.h>
#include <Arduino.h>

#define MINUS 700
#define MAXUS 2300
#define LEFTPIN 26
#define RIGHTPIN 27
#define SERVOFREQ 50

#define WHEELRADIUS_M 1
#define INITIAL_LENGTH_M 0.00

#define SERVO_SPEED_RAD_SEC 6.98412
#define SERVO_SPEED_M_SEC SERVO_SPEED_RAD_SEC * WHEELRADIUS_M
#define SERVO_SPEED_VALUE_ANGLE 20


typedef struct {
  double length1;
  double length2;
} SteeringData;

//a queue to send the data to the steering function
extern TaskHandle_t _steeringTask = NULL;
extern QueueHandle_t steeringQueue;

void motorSetup();
void steering(void *pvParameters);

#endif