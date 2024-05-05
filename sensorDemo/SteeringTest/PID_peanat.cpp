/* 
 * PID library made for robots!
 * Extra: custom initial output, deadband limits
 * Date created: Aug. 16, 2022
 */

#include "Arduino.h"
#include "PID_peanat.h"

const double SECS_IN_MS = 0.001;


/* 
 * Initialize PID object.
 *
 * @param user_kp: proportional coefficient, should be >= 0
 * @param user_ki: integral coefficient, should be >= 0
 * @param user_kd: derivative coefficient, should be >= 0
 * 
 * @returns PID object
 */
PID::PID(double user_kp, double user_ki, double user_kd) {
    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
    init_output = 0.0;
    reverse = false;
    sample_time = 100;
    min_output = 0.0;
    max_output = 255.0;
    deadband = 0.0;

    // Reverse coefficients if in reverse mode
    if (reverse == true) {
        kp *= -1.0;
        ki *= -1.0;
        kd *= -1.0;
    }

    // Initiate other fields
    first_run = true;
    cumulative_error = 0.0;
}

/*
 * Updates PID coefficients.
 * 
 * @param user_kp: proportional coefficient, should be >= 0
 * @param user_ki: integral coefficient, should be >= 0
 * @param user_kd: derivative coefficient, should be >= 0
 * 
 * @returns none
 */
void PID::updateCoeffs(double user_kp, double user_ki, double user_kd) {
    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
}

/*
 * Set initial output; must be within or on output bounds.
 *
 * @param user_init_output: desired initial output, for when we want the output to start at a specific value
 * 
 * @returns none
 */
void PID::setInitOutput(double user_init_output) {
    init_output = user_init_output;
}

/*
 * Toggle reverse mode on or off.
 *
 * @param user_reverse: true to turn reverse mode on, false to turn off
 * 
 * @returns none
 */
void PID::setReverse(bool user_reverse) {
   if (reverse != user_reverse) {
        reverse = user_reverse;
        kp *= -1.0;
        ki *= -1.0;
        kd *= -1.0;
   }
}

/*
 * Set sample time (recommended < 100).
 * 
 * @param user_sample_time: sample time in ms
 * 
 * @returns none
 */
void PID::setSampleTime(unsigned long user_sample_time) {
    sample_time = user_sample_time;
}

/*
 * Set output bounds; useful if something like a servo has input limits.
 * Make sure that the initial output is on or within these bounds!
 *
 * @param user_min_output: minimum output value (inclusive)
 * @param user_max_output: maximum output value (inclusive)
 * 
 * @returns none
 */
void PID::setOutputBounds(double user_min_output, double user_max_output) {
    min_output = user_min_output;
    max_output = user_max_output;
}

/*
 * Set deadband range (difference between previous and current calculated output, in which the 
 * previous output is returned) if we need to prevent mechanical wear. Deadband must be less
 * than max output - min output.
 * 
 * @param user_deadband: deadband range, in units of output
 * 
 * @returns none
 */
void PID::setDeadband(double user_deadband) {
    deadband = user_deadband;
}

/*
 * Computes PID output based on standard PID algorithm; implements deadband functionality and
 * measures to prevent integral windup. The first run will output the initial output value.
 * This loop can run for a maximum of 49 days before unsigned long overflows.
 * 
 * @param user_setpoint: desired value of something (e.g. servo position)
 * @param user_input: the acutal value
 * 
 * @returns output to be fed back into the controller
 */
double PID::compute(double user_setpoint, double user_input) {
    // If first run, initialize variables
    if (first_run == true) {
        setpoint = user_setpoint;
        input = user_input;

        prev_time = millis();
        prev_error = setpoint - input;
        prev_output = init_output;

        output = init_output;
        first_run = false;
    }

    // Subsequent runs
    else {
        current_time = millis();
        unsigned long time_interval = current_time - prev_time;
        
        if (time_interval >= sample_time) {
            setpoint = user_setpoint;
            input = user_input;
            double error = setpoint - input;

            // Proportional
            output = init_output + kp * error;
            // Integral
            cumulative_error += ki * error * ((double)time_interval * SECS_IN_MS);
            cumulative_error = min(max(cumulative_error, min_output), max_output); // Limit cumulative error to prevent windup
            output += cumulative_error;
            // Derivative
            output += kd * (prev_error - error) / ((double)time_interval * SECS_IN_MS);

            // Limit output
            output = min(max(output, min_output), max_output);

            // Don't change output if within deadband
            if (fabs(output - prev_output) < deadband) {
                output = prev_output;
            }

            // For the next run
            prev_time = current_time;
            prev_error = error;
            prev_output = output;
        }
    }

    return output;
}

double PID::getOutput() {
    return output;
}

double PID::getkp() {
    return kp;
}

double PID::getki() {
    return ki;
}

double PID::getkd() {
    return kd;
}
