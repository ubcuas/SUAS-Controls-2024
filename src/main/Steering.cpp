#include "Steering.h"
#include "Arduino.h"
#include <Servo.h>
#include <ESP32Servo.h>


//Define servo motor classes
Servo leftMotor;
Servo rightMotor;
//Change Pulse Width frequency as needed
#define minUs 1000; 
#define maxUs 2000;
//PWM object
ESP32PWM pwm;

void motorSetup(){
  leftMotor.attach(11,minUs,maxUs);
  rightMotor.attach(12,minUs,maxUs);
  pwm.attachPin(11,1000); //1Khz - Is this correct?
  pwm.attachPin(12,1000); //1Khz - Is this correct?
  leftMotor.setPeriodHertz(50);      // Standard 50hz servo - Is this correct?
	rightMotor.setPeriodHertz(50);      // Standard 50hz servo - Is this correct?
}

void steering(int yaw){
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