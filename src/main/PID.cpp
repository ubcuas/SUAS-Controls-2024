/*
 * Author: Bobsy Narayan
 * Date: Feb 24, 2024
 * Notes: 
 *          Payload Auto Parachute 2023-2024
 *          Parachute Steering PID Controller Header File    
 *          Bunch of functions to Initialize, calculate the controller and to change the setpoint. 
 * Teensy 4.0
 * 
 * Extra comments added in for personal development
 * 
 * Taken from PIDSelfSteer.Cpp from AUVSI Rover 2021-2022
 * 
 * 
 */

#include "PID.h"
/*
 * PID Initalize function
 * Parameters: setpoint = input setpoint
 *             control frequency = acquire rate of PID
 */
void Pid::PIDInit(double controlFrequency){
    DT=1/controlFrequency;
}

/*
 * PID Calculate function
 * Parameters:processVariable - The reading from the sensor (feedback signal)
 */
double Pid::PIDcalculate(double processVariable){
    // Set Error
    error = setpoint - processVariable;
    
    // Calculate the terms: K, I, and D - Do I need to adjust anything for this for yaw angle? 2)!!
    double KTerm = error * KP;
    double ITerm = (integral + error * DT) * KI;
    double DTerm = ((error - prevError) / DT) * KD;

    // Compute Output
    output = KTerm + ITerm + DTerm;
    
    // Check if the output is within limits
    if(output < minPIDVal){
      output = minPIDVal;
    }
    if(output > maxPIDVal){
      output = maxPIDVal;
    }
    
    // Set Integral and Previous Error
    prevError = error;
    integral += error * DT;

    return output;
}

/*
 * Function to Update the setpoint for the controller
 * Parameters: PIDstructure - A pointer to an array storing the state of the controller
 *             setpoint     - The new value of the setpoint
 */
void Pid::PIDupdateSetpoint(double setpoint){
    this->setpoint = setpoint; 
}