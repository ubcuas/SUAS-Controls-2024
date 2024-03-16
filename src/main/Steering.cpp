#include "Steering.h"
#include "Arduino.h"



//Define servo motor classes
Servo leftMotor;
Servo rightMotor;


//PWM object
ESP32PWM pwm;

void motorSetup(){
  leftMotor.attach(LEFTPIN,MINUS,MAXUS);
  rightMotor.attach(RIGHTPIN,MINUS,MAXUS);
  pwm.attachPin(LEFTPIN,ATTACHFREQ); //1Khz - Is this correct?
  pwm.attachPin(RIGHTPIN,ATTACHFREQ); //1Khz - Is this correct?
  leftMotor.setPeriodHertz(SERVOFREQ);      // Standard 50hz servo - Is this correct?
	rightMotor.setPeriodHertz(SERVOFREQ);      // Standard 50hz servo - Is this correct?
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