// #include "advancedSteering.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "encoder.h"
#include "encoder_steering.h"

double desired_heading = 0;
double current_heading = 0;

QueueHandle_t steeringQueue = NULL;

void setup() {
  Serial.begin(921600); // Start serial communication at 921600 baud rate

  pinMode(ENC_1, INPUT);
  attachInterrupt(ENC_1, enc_1_isr, RISING);
  pinMode(ENC_2, INPUT);
  attachInterrupt(ENC_2, enc_2_isr, RISING);

  // Create a queue that can hold 1 item of double type
  steeringQueue = xQueueCreate(1, sizeof(double));
  if (steeringQueue == NULL) {
    Serial.println("Failed to create steering queue");
    return; // Early exit if queue creation fails
  }

  steering_setup();
}

void loop() {
  // Check if we have incoming data
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');  // Read the incoming data until newline
    inputString.trim();  // Trim whitespace and newline characters

    // Parse the string to extract desired and current headings
    double incomingDesiredHeading, incomingCurrentHeading;
    if (sscanf(inputString.c_str(), "%lf,%lf", &incomingDesiredHeading, &incomingCurrentHeading) == 2) {
      // Successfully parsed two doubles
      // Update the global variables with the new data
      desired_heading = incomingDesiredHeading * 0.01745; // Convert to radians
      current_heading = incomingCurrentHeading * 0.01745; // Convert to radians
      Serial.printf("Desired heading: %lf, Current heading: %lf\n", desired_heading, current_heading);
    } else {
      Serial.println("Invalid format or incomplete data");
    }
  }

  Serial.printf("Count: %d %d\n", count_1, count_2);
  delay(100);
}