// Steering with feedback from encoders

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "PID_peanat.h"
#include "encoder.h"
#include "encoder_steering.h"

// Declare servo motor objects
Servo servo_1;
Servo servo_2;

// Initialize PID
PID pid(50.0, 0, 0);

void steering_setup() {
  pid.setOutputBounds(-150, 150); // Limit to 150 mm extension/retraction
  pid.setSampleTime(10); // ms
  // pid.setDeadband(DIST_PER_TICK);

  servo_1.attach(SERVO_1, MIN_US, MAX_US);
  servo_1.setPeriodHertz(SERVO_FREQ);
  servo_2.attach(SERVO_2, MIN_US, MAX_US);
  servo_2.setPeriodHertz(SERVO_FREQ);
  // No spin
  servo_1.write(90);
  servo_2.write(90);

  // Start task
  xTaskCreatePinnedToCore(
    servoControlTask,       // Task function
    "ServoControlTask",     // Task name
    4096,                   // Stack size (words)
    NULL,                   // Task parameters
    -1,                     // Priority (1 is default)
    NULL,                   // Task handle
    0                       // Core number (0 or 1)
  );
}


// Get difference between two angles in radians [-pi, pi)
double angle_diff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff >= PI) {
    diff -= 2 * PI;
  }
  while (diff < -PI) {
    diff += 2 * PI;
  }
  return diff;
}


/* 
 * Function to control servos
 * @param des_heading
 * @param current_heading
 */
void servo_control(double des_heading, double current_heading) {
  SteeringData des_lengths;
  SteeringData current_lengths = {(double)DIST_PER_TICK*((int)count_1/2), (double)DIST_PER_TICK*((int)count_2/2)};

  double input = angle_diff(des_heading, current_heading);
  Serial.printf("Input: %lf\n", input);
  if (fabs(input) < FORWARD_THRESH) { // Go forward if we don't need to change our heading that much
    des_lengths.l1 = -100.0; // Servo 1
    des_lengths.l2 = -100.0; // Servo 2
  }
  else {
    double output = pid.compute(0.0, input);
    // Serial.printf("Output: %lf\n", output);
    des_lengths.l1 = output;
    des_lengths.l2 = -output;
  }

  Serial.printf("Lengths: %lf %lf\n", des_lengths.l1, des_lengths.l2);

  // Servo 1
  if (des_lengths.l1 - current_lengths.l1 > (DIST_PER_TICK)) {
    servo_1.write(90 + SERVO_SPEED_VALUE_ANGLE);
    forward_1 = true;
  } else if (des_lengths.l1 - current_lengths.l1 < -(DIST_PER_TICK)) {
    servo_1.write(90 - SERVO_SPEED_VALUE_ANGLE);
    forward_1 = false;
  } else {
    servo_1.write(90);
  }

  // Servo 2
  if (des_lengths.l2 - current_lengths.l2 > (DIST_PER_TICK)) {
    servo_2.write(90 - SERVO_SPEED_VALUE_ANGLE);
    forward_2 = true;
  } else if (des_lengths.l2 - current_lengths.l2 < -(DIST_PER_TICK)) {
    servo_2.write(90 + SERVO_SPEED_VALUE_ANGLE);
    forward_2 = false;
  } else {
    servo_2.write(90);
  }

}


void servoControlTask(void *pvParameters) {
    (void) pvParameters;
    
    while (true) {
        // Call servo_control function here
        servo_control(desired_heading, current_heading); // Modify parameters as needed
        
        // Delay for the desired period (in milliseconds)
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
    }
}


