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

#ifndef _PI_TEST_H    
#define _PI_TEST_H

#define MAX_OUTPUT 255.0
#define MIN_OUTPUT -255.0
#define DEFAULT_KP 1.0
#define DEFAULT_KI 1.8
#define DEFAULT_KD 0.6

class PID {
public:
    // Empty constructor
    PID();

    // Function to initialize the PID controller with specific parameters
    void PIDInit(double setpoint, double controlFrequency);

    // Calculate the PID output based on the process variable
    void PIDcalculate(double processVariable);

    // Update the setpoint of the PID controller
    void PIDupdateSetpoint(double setpoint);

private:
  double DT;
  double maxPIDVal = MAX_OUTPUT;
  double minPIDVal = MIN_OUTPUT;
  double KP = DEFAULT_KP;
  double KI = DEFAULT_KI;
  double KD = DEFAULT_KD;
  double setpoint=0;
  double error=0;
  double output=0;
  double prevError=0;
  double integral=0;
};

/*
  
*/

#endif