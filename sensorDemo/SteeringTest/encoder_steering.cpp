// Steering with feedback from encoders

#include <Arduino.h>
#include <ESP32Servo.h>
#include "PID_peanat.h"
#include "encoder.h"
#include "encoder_steering.h"

// Declare servo motor objects
Servo servo_1;
Servo servo_2;

// Initialize PID
PID pid(10.0, 0, 0);

void steering_setup() {
  pid.setOutputBounds(0, 150); // Limit to 150 mm extension/retraction
  pid.setSampleTime(10); // ms
  pid.setDeadband(DIST_PER_TICK);

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
  SteeringData current_lengths = {DIST_PER_TICK*count_1, DIST_PER_TICK*count_2};

  double input = angle_diff(des_heading, current_heading);
  if (input < FORWARD_THRESH) { // Go forward if we don't need to change our heading that much
    des_lengths.l1 = -100.0; // Servo 1
    des_lengths.l2 = -100.0; // Servo 2
  }
  else {
    double output = pid.compute(0.0, input);
    des_lengths.l1 = output;
    des_lengths.l2 = -output;
  }

  // Write to servos
  int val_1 = 90 + (des_lengths.l1 > current_lengths.l1 ? SERVO_SPEED_VALUE_ANGLE : (des_lengths.l1 < current_lengths.l1 ? -SERVO_SPEED_VALUE_ANGLE : 0));
  servo_1.write(val_1);
  int val_2 = 90 + (des_lengths.l2 > current_lengths.l2 ? SERVO_SPEED_VALUE_ANGLE : (des_lengths.l2 < current_lengths.l2 ? -SERVO_SPEED_VALUE_ANGLE : 0));
  servo_2.write(val_2);
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


