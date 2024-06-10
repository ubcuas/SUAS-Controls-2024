#ifndef ENCODER_STEERING_H
#define ENCODER_STEERING_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "PID_peanat.h"
#include "encoder.h"

#define SERVO_1 26
#define SERVO_2 27

#define MIN_US 700
#define MAX_US 2300
#define SERVO_FREQ 50 // Standard 50hz servo
#define SERVO_SPEED_VALUE_ANGLE 80
#define PI 3.14159

static const double DRUM_DIAMETER = 22; // mm
static const double DIST_PER_TICK = (double)PI*DRUM_DIAMETER / 6.0; // mm (There are 4 markers around the wheel)
static const double FORWARD_THRESH = 10.0 * (double)PI/180.0; // rad (Go forward if we are within 10 degrees)

typedef struct {
  double l1;
  double l2;
} SteeringData;

typedef struct {
  double desiredYaw;
  double currentYaw;
  double desiredForward;
} AngleData;

extern TaskHandle_t _servoControlTask;
extern QueueHandle_t steeringQueue;

extern double desired_heading;
extern double current_heading;
extern volatile bool forward_1;
extern volatile bool forward_2;

void steering_setup(double acquireRate, double kp, double ki, double kd);
double angle_diff(double angle1, double angle2);
void  servo_control(AngleData data);
void servoControlTask(void *pvParameters);
void sendSteeringData(AngleData data);
void do_nothing();

#endif