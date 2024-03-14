/*Autonomous Parachute Steering Functon Header
 *Author: Bobsy Narayan and Nischay Joshi
 *Edited: Feb 19, 2022
 */
//Steering function declarations
#ifndef _STEERING_H    
#define _STEERING_H   
#include <ESP32Servo.h>

#define minUs 1000 
#define maxUs 2000
#define leftPin 11
#define rightPin 12
#define attachFreq 1000
#define servoFreq 50

void motorSetup();
void steering(double yaw);

#endif