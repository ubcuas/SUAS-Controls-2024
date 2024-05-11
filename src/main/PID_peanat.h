#ifndef PID_PEANAT_H
#define PID_PEANAT_H

/*
 * This is a PID class to be used in PID control loops.
 */

class PID {

    public:

    // Constructor
    PID(double user_kp, double user_ki, double user_kd);

    // Change settings
    void updateCoeffs(double user_kp, double user_ki, double user_kd);
    void setInitOutput(double user_init_output);
    void setReverse(bool user_reverse);
    void setSampleTime(unsigned long user_sample_time);
    void setOutputBounds(double user_min_output, double user_max_output);
    void setDeadband(double user_deadband);

    // Calculate PID output
    double compute(double user_setpoint, double user_input);

    // Get private variables
    double getOutput();
    double getkp();
    double getki();
    double getkd();


    private:

    double setpoint;                        // Target value
    
    double input;                           // What the value really is

    double output;                          // Calculated output
    double init_output;                     // Output value to start from
    double prev_output;                     // Previous output

    double kp;                              // Proportional coefficient
    double ki;                              // Integral coefficient
    double kd;                              // Derivative coefficient

    double prev_error;                      // Previous error
    double cumulative_error;                // Cumulative error for calculating integral output

    unsigned long current_time;             // Current time in milliseconds
    unsigned long prev_time;                // Time at which previous input was taken
    unsigned long sample_time;              // Input sample time

    double min_output;                      // Max output
    double max_output;                      // Min output

    double deadband;                        // The range in which we don't change the output (to prevent mechanical wear)
    
    bool reverse;                           // Reverse mode (if true, output * -1)
    bool first_run;                         // True if it is the very first call to the `compute` function
};

#endif