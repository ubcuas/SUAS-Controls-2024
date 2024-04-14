#include "advancedSteering.h"

void setup() {
    Serial.begin(921600); // Start serial communication at 921600 baud rate
    steeringQueue = xQueueCreate(1, sizeof(SteeringData)); // Create a queue that can hold 1 item of SteeringData type

    if (steeringQueue == NULL) {
        Serial.println("Failed to create steering queue");
        return; // Early exit if queue creation fails
    }

    motorSetup(); // Setup motors and start the steering task
}

void loop() {
    // Check if we have incoming data
    if (Serial.available()) {
        String inputString = Serial.readStringUntil('\n');  // Read the incoming data until newline
        inputString.trim();  // Trim whitespace and newline characters

        // Declare a variable to hold the parsed data
        SteeringData incomingData;

        // Parse the string to extract lengths
        if (sscanf(inputString.c_str(), "%lf,%lf", &incomingData.length1, &incomingData.length2) == 2) {
            // Successfully parsed two doubles
            // Overwrite the current value in the queue with the new data
            incomingData.length1 *= 0.01745;
            incomingData.length2 *= 0.01745;
            Serial.printf("Length1: %lf\tLength2: %lf\n", incomingData.length1, incomingData.length2);
            if (xQueueOverwrite(steeringQueue, &incomingData) != pdPASS) {
                Serial.println("Failed to overwrite data in steering queue");
            }
        } else {
            Serial.println("Invalid format or incomplete data");
        }
    }
}