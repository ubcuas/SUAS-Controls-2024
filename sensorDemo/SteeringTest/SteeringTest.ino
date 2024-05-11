// #include "advancedSteering.h"
#include <Arduino.h>
#include "encoder_steering.h"

double desired_heading = 0;
double current_heading = 0;



void setup() {
  Serial.begin(921600); // Start serial communication at 921600 baud rate
  steering_setup(100.0, 50.0, 0, 0); // Initialize steering
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
    // create the data packet
    AngleData data = {desired_heading, current_heading, 0};
    // send the data packet to the queue
    sendSteeringData(data);
  }

  Serial.printf("Count: %d %d\n", count_1, count_2);
  delay(100);
}