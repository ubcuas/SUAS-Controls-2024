#include "ParachuteSteering.h"
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
  //Value written to motors - needs to be adjusted using yaw value. Unsure how to do this - 1!!
  int right_motor_value = map(yaw, -180, 180, 0, 180);
  int left_motor_value = map(yaw, -180, 180, 0, 180);
 
 
  Serial.print(left_motor_value);Serial.print("\t");
  Serial.print(right_motor_value);Serial.print("\n");
  
  //Making sure values are always within motor range:
  left_motor_value = constrain(left_motor_value, 0, 180);
  right_motor_value = constrain(right_motor_value, 0, 180);
 
  leftMotor.write(left_motor_value);
  rightMotor.write(right_motor_value);
  delay(100);
 

//Uncomment to get the values

  
 // Serial.print(right_motor_value);Serial.print("\t");
 // Serial.print(left_motor_value);Serial.print("\n");
  





  
}