/*
 * Author: Nischay Joshi/Bobsy Narayan
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
 * Note-Consider replacing numbers with enums for clarity
 */



/*
PID Structure MAP: [ 0      1       2       3       4       5       6            7         8         9             10]      
                     dt    max     min      kp      ki      kd     setpoint     error    output   preverror     integral
*/

#include "PI_Test.h"

#define MAX_OUTPUT 255.0
#define MIN_OUTPUT -255.0
#define DEFAULT_KP 1.0
#define DEFAULT_KI 1.8
#define DEFAULT_KD 0.6

/*
 * Initialize PID Function
 * Parameters: PIDstructure     - A pointer to an array of doubles storing the current status of the controller 
 *             setpoint         - Value at which we need to set the controller
 *             controlFrequency - The frequency at which the controller is running
 */
void PIDInit(double *PIDstructure, double setpoint, double controlFrequency){
    PIDstructure[DT_INDEX] = 1/controlFrequency;
    PIDstructure[MAX_INDEX] = MAX_OUTPUT;
    PIDstructure[MIN_INDEX] = MIN_OUTPUT;
    PIDstructure[KP_INDEX] = DEFAULT_KP;
    PIDstructure[KI_INDEX] = DEFAULT_KI;
    PIDstructure[KD_INDEX] = DEFAULT_KD;
    PIDstructure[SETPOINT_INDEX] = setpoint;
    PIDstructure[ERROR_INDEX] = 0.0;
    PIDstructure[OUTPUT_INDEX] = 0.0;
    PIDstructure[PREV_ERROR_INDEX] = 0.0;
    PIDstructure[INTEGRAL_INDEX] = 0.0;
}

/*
 * PID Calculate function
 * Parameters: PIDstructure    - A pointer to an array storing the current state of the controller
 *             processVariable - The reading from the sensor (feedback signal)
 */
void PIDcalculate(double *PIDstructure, double processVariable){
    // Set Error
    PIDstructure[ERROR_INDEX] = PIDstructure[SETPOINT_INDEX] - processVariable;
    
    // Calculate the terms: K, I, and D - Do I need to adjust anything for this for yaw angle? 2)!!
    double KTerm = PIDstructure[ERROR_INDEX] * PIDstructure[KP_INDEX];
    double ITerm = (PIDstructure[INTEGRAL_INDEX] + PIDstructure[ERROR_INDEX] * PIDstructure[DT_INDEX]) * PIDstructure[KI_INDEX];
    double DTerm = ((PIDstructure[ERROR_INDEX] - PIDstructure[PREV_ERROR_INDEX]) / PIDstructure[DT_INDEX]) * PIDstructure[KD_INDEX];

    // Compute Output
    PIDstructure[OUTPUT_INDEX] = KTerm + ITerm + DTerm;
    
    // Check if the output is within limits
    if(PIDstructure[OUTPUT_INDEX] < PIDstructure[MIN_INDEX]){
      PIDstructure[OUTPUT_INDEX] = PIDstructure[MIN_INDEX];
    }
    if(PIDstructure[OUTPUT_INDEX] > PIDstructure[MAX_INDEX]){
      PIDstructure[OUTPUT_INDEX] = PIDstructure[MAX_INDEX];
    }
    
    // Set Integral and Previous Error
    PIDstructure[PREV_ERROR_INDEX] = PIDstructure[ERROR_INDEX];
    PIDstructure[INTEGRAL_INDEX] += PIDstructure[ERROR_INDEX] * PIDstructure[DT_INDEX];
}

/*
 * Function to Update the setpoint for the controller
 * Parameters: PIDstructure - A pointer to an array storing the state of the controller
 *             setpoint     - The new value of the setpoint
 */
void updateSetpoint( double *PIDstructure, double setpoint){
    PIDstructure[SETPOINT_INDEX] = setpoint; 
}