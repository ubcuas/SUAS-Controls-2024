/*Autonomous Parachute Steering Functon Header
 *Author: Bobsy Narayan and Nischay Joshi
 *Edited: Feb 19, 2022
 */
//Steering function declarations
#ifndef _STEERING_H    
#define _STEERING_H   
#include <ESP32Servo.h>

#define MINUS 1000 
#define MAXUS 2000
#define LEFTPIN 11
#define RIGHTPIN 12
#define ATTACHFREQ 1000
#define SERVOFREQ 50

void motorSetup();
void steering(double yaw);

#endif