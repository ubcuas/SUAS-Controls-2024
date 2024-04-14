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

// Define the global variables
TaskHandle_t _steeringTask = NULL;
QueueHandle_t steeringQueue = NULL;

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

    double currentLength1 = desiredLengths.length1;
    double currentLength2 = desiredLengths.length2;

    double desiredLength1 = currentLength1;
    double desiredLength2 = currentLength2;

    double errorLength1 = 0;
    double errorLength2 = 0;

    while(1){
        //if we have data in the queue, if not, keep the previous value
        if (xQueueReceive(steeringQueue, &desiredLengths, 0) == pdTRUE){
          Serial.printf("Servo1: %lf\tServo2: %lf\n", desiredLengths.length1, desiredLengths.length2);
          desiredLength1 = desiredLengths.length1;
          desiredLength2 = desiredLengths.length2;
        }
        //update the distance to travel
        errorLength1 = desiredLength1 - currentLength1;
        errorLength2 = desiredLength2 - currentLength2;

        if(abs(errorLength1) > ERROR_DEADZONE || abs(errorLength2) > ERROR_DEADZONE){
          //calculate the amount of time to move the servo
          double time1 = abs(errorLength1) / SERVO_SPEED_M_SEC * 1000;
          double time2 = abs(errorLength2) / SERVO_SPEED_M_SEC * 1000;

          //move the servo
          int leftMotorvalue = 90 + (errorLength1 > 0 ? SERVO_SPEED_VALUE_ANGLE : -SERVO_SPEED_VALUE_ANGLE);
          int rightMotorvalue = 90 + (errorLength2 > 0 ? SERVO_SPEED_VALUE_ANGLE : -SERVO_SPEED_VALUE_ANGLE);

          if(abs(errorLength1) < ERROR_DEADZONE){
            leftMotorvalue = 90;
          }
          if(abs(errorLength2) < ERROR_DEADZONE){
            rightMotorvalue = 90;
          }

          leftMotor.write(leftMotorvalue);
          rightMotor.write(rightMotorvalue);
          Serial.printf("CurrentLeng: %lf\tDesiredLength %lf\tError: %lf\tTime1: %lf\tServoValue: %d\n", currentLength1, desiredLength1,  errorLength1, time1, leftMotorvalue);
          unsigned long startTime = millis();

          //wait for the servo to move
          bool leftMotordone = false;
          bool rightMotordone = false;

          int i = 0;
          while(1){
            
              if(!leftMotordone && ((millis() - startTime) > time1)){
                  leftMotordone = true;
                  leftMotor.write(90);  // Reset servo to neutral position after moving
                  Serial.println("Servo Left done");
              }
              if(!rightMotordone && ((millis() - startTime) > time2)){
                  rightMotordone = true;
                  rightMotor.write(90);  // Reset servo to neutral position after moving
                  Serial.println("Servo right done");
              }
              if(leftMotordone && rightMotordone){
                  break;  // Exit loop if both servos have completed their movements
              }
              // Wait for 1 ms to prevent hogging the CPU
              // vTaskDelay(1 / portTICK_PERIOD_MS);
              delay(1);
              Serial.println(i++);
          }

          leftMotor.write(90);
          rightMotor.write(90);
          // Update the current lengths
          currentLength1 = desiredLengths.length1;
          currentLength2 = desiredLengths.length2;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);

    }
}