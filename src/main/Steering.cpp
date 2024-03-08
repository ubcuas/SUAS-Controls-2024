#include "Steering.h"
#include "Arduino.h"
#include <Servo.h>


//Define servo motor classes
Servo leftMotor;
Servo rightMotor;

void motorSetup(){
  leftMotor.attach(11);
  rightMotor.attach(12);
}

void steering(int yaw){
  //This part needs to be fixed. Euler->PID->steering. Change PID to be an angle or multiply value here to get angle?
  int left_motor_value = constrain(left_motor_value, 0, 180);
  int right_motor_value = constrain(right_motor_value, 0, 180);
 
 
  Serial.print(left_motor_value);Serial.print("\t");
  Serial.print(right_motor_value);Serial.print("\n");
 
  leftMotor.write(left_motor_value);
  rightMotor.write(right_motor_value);
  delay(100);
 

//Uncomment to get the values
 // Serial.print(right_motor_value);Serial.print("\t");
 // Serial.print(left_motor_value);Serial.print("\n");
  





  
}