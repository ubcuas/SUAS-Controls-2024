#ifndef ENCODER_STEERING_H
#define ENCODER_STEERING_H

#define SERVO_1 26
#define SERVO_2 27

#define MIN_US 700
#define MAX_US 2300
#define SERVO_FREQ 50 // Standard 50hz servo
#define SERVO_SPEED_VALUE_ANGLE 80
#define PI 3.14159

static const double DRUM_DIAMETER = 2.2; // mm
static const double DIST_PER_TICK = PI*DRUM_DIAMETER / 4; // mm (There are 4 markers around the wheel)
static const double FORWARD_THRESH = 10.0 * PI/180; // rad (Go forward if we are within 10 degrees)

typedef struct {
  double l1;
  double l2;
} SteeringData;

extern double desired_heading;
extern double current_heading;

void steering_setup();
double angle_diff(double angle1, double angle2);
void servo_control(double des_heading, double current_heading);
void servoControlTask(void *pvParameters);

#endif