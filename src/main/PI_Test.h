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
 */

/*
Those are called #include guards.
Once the header is included, it checks if a unique value (in this case HEADERFILE_H) is defined. Then if it's not defined, it defines it and continues to the rest of the page.
When the code is included again, the first ifndef fails, resulting in a blank file.
That prevents double declaration of any identifiers such as types, enums and static variables.
SRC: https://stackoverflow.com/questions/1653958/why-are-ifndef-and-define-used-in-c-header-files
*/

#ifndef _PI_TEST_H    
#define _PI_TEST_H

enum {
    DT_INDEX,
    MAX_INDEX,
    MIN_INDEX,
    KP_INDEX,
    KI_INDEX,
    KD_INDEX,
    SETPOINT_INDEX,
    ERROR_INDEX,
    OUTPUT_INDEX,
    PREV_ERROR_INDEX,
    INTEGRAL_INDEX,
    NUM_ELEMENTS // This should always be the last element to represent the number of elements in the enum
};

void PIDInit(double *PIDstructure, double setpoint, double controlFrequency);
void PIDcalculate(double *PIDstructure, double processVariable);
void updateSetpoint( double *PIDstructure, double setpoint);

#endif