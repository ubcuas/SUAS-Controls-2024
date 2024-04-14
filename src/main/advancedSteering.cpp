/*
 * Autonomous Parachute Steering Functon Implementation
 * Keeping track of the angle of a continous rotation servo
 * Author: Nischay Joshi
 * Created: April 13th, 2024
 */

#include "advancedSteering.h"

//Declare servo motor objects
Servo leftMotor;
Servo rightMotor;

void motorSetup(){
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    leftMotor.attach(LEFTPIN,MINUS,MAXUS);
    rightMotor.attach(RIGHTPIN,MINUS,MAXUS);
    leftMotor.setPeriodHertz(SERVOFREQ);      // Standard 50hz servo
    rightMotor.setPeriodHertz(SERVOFREQ);      // Standard 50hz servo
    //set at 0 speed value
    leftMotor.write(90);
    rightMotor.write(90);

    //start the servo control task
    BaseType_t ret = xTaskCreatePinnedToCore(
        steering, /* Function to implement the task */
        "steeringTask", /* Name of the task */
        4096,  /* Stack size in words */
        NULL,  /* Task input parameter */
        configMAX_PRIORITIES - 1,  /* Priority of the task */
        &_steeringTask,  /* Task handle. */
        0); /* Core where the task should run */
    if (ret != pdPASS) {
        Serial.println("Failed to create steering task");
    }
    else {
        Serial.println("Steering task created");
    }
}

//steering function to control the steering of the parachute
void steering(void *pvParameters){
    SteeringData desiredLengths;
    desiredLengths.length1 = INITIAL_LENGTH_M; 
    desiredLengths.length2 = INITIAL_LENGTH_M;

    double currentLength1 = data.length1;
    double currentLength2 = data.length2;

    double errorLength1 = 0;
    double errorLength2 = 0;

    while(1){
        //if we have data in the queue, if not, keep the previous value
        if (xQueueReceive(steeringQueue, &desiredLengths, 0) == pdTRUE){
        }
        //update the distance to travel
        errorLength1 = desiredLengths.length1 - currentLength1;
        errorLength2 = desiredLengths.length2 - currentLength2;

        //calculate the amount of time to move the servo
        double time1 = abs(errorLength1) / SERVO_SPEED_M_SEC;
        double time2 = abs(errorLength2) / SERVO_SPEED_M_SEC;

        //move the servo
        int servo1value = 90 + (errorLength1 > 0 ? SERVO_SPEED_VALUE_ANGLE : -SERVO_SPEED_VALUE_ANGLE);
        int servo2value = 90 + (errorLength2 > 0 ? SERVO_SPEED_VALUE_ANGLE : -SERVO_SPEED_VALUE_ANGLE);

        servo1.write(servo1value);
        servo2.write(servo2value);
        unsigned long startTime = millis();

        //wait for the servo to move
        bool servo1done = false;
        bool servo2done = false;
        while(1){
            if(!servo1done && (millis() - startTime > time1)){
                servo1done = true;
                servo1.write(90);  // Reset servo to neutral position after moving
            }
            if(!servo2done && (millis() - startTime > time2)){
                servo2done = true;
                servo2.write(90);  // Reset servo to neutral position after moving
            }
            if(servo1done && servo2done){
                break;  // Exit loop if both servos have completed their movements
            }
            // Wait for 1 ms to prevent hogging the CPU
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        // Update the current lengths
        currentLength1 = desiredLengths.length1;
        currentLength2 = desiredLengths.length2;

        vTaskDelay(1 / portTICK_PERIOD_MS);

    }
}