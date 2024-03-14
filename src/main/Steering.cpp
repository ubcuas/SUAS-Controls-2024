#include "Steering.h"
#include "Arduino.h"



//Define servo motor classes
Servo leftMotor;
Servo rightMotor;


//PWM object
ESP32PWM pwm;

void motorSetup(){
  leftMotor.attach(rightPin,minUs,maxUs);
  rightMotor.attach(leftPin,minUs,maxUs);
  pwm.attachPin(leftPin,attachFreq); //1Khz - Is this correct?
  pwm.attachPin(rightPin,attachFreq); //1Khz - Is this correct?
  leftMotor.setPeriodHertz(servoFreq);      // Standard 50hz servo - Is this correct?
	rightMotor.setPeriodHertz(servoFreq);      // Standard 50hz servo - Is this correct?
}

void steering(double yaw){
  //This part needs to be fixed. Euler->PID->steering. Change PID to be an angle or multiply value here to get angle?
  int left_motor_value = constrain(left_motor_value, 0, 180);
  int right_motor_value = constrain(right_motor_value, 0, 180);
 
 
  Serial.print(left_motor_value);Serial.print("\t");
  Serial.print(right_motor_value);Serial.print("\n");
 
  leftMotor.write(left_motor_value);
  rightMotor.write(right_motor_value);
 

//Uncomment to get the values
 // Serial.print(right_motor_value);Serial.print("\t");
 // Serial.print(left_motor_value);Serial.print("\n");
  
}