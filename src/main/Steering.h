/*Autonomous Parachute Steering Functon Header
 *Author: Bobsy Narayan and Nischay Joshi
 *Edited: Feb 19, 2022
 */
//Steering function declarations
#ifndef _STEERING_H    
#define _STEERING_H   
#include <ESP32Servo.h>

#define MINUS 450
#define MAXUS 2450
#define LEFTPIN 26
#define RIGHTPIN 27
#define ATTACHFREQ 1000
#define SERVOFREQ 50

void motorSetup();
void steering(double yaw);

#endif