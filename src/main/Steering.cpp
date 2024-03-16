#include "Steering.h"
#include "Arduino.h"



//Define servo motor classes
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
  leftMotor.write(90);
  rightMotor.write(90);
}

void steering(double yaw){
  //This part needs to be fixed. Euler->PID->steering. Change PID to be an angle or multiply value here to get angle?
  //-x to +x ->Map from 0 to 180 ->change double to int
  //-x to +x ->Map from 0 to 180 ->change double to int
  //pv is the value that we observe, setpoint is what we want course to be.
  //Set setpoint, then update pv.
  int left_motor_value = map(yaw,-1000,1000,0,180);
  int right_motor_value = map(yaw,-1000,1000,0,180);
 
 
  Serial.print(left_motor_value);Serial.print("\t");
  Serial.print(right_motor_value);Serial.print("\n");
 
  leftMotor.write(left_motor_value);
  rightMotor.write(right_motor_value);
 

//Uncomment to get the values
 // Serial.print(right_motor_value);Serial.print("\t");
 // Serial.print(left_motor_value);Serial.print("\n");
  
}