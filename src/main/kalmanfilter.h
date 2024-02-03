/*
    kalmanfilter.h - Header file for Kalman filter class
    Name: Nischay Joshi
    Date: 08-12-23
    Kalman Filter implementationn refered from Alex Becker - Kalman Filter from Ground Up
    
*/


/*
        Algortihm:

        1. initialize the vectors and matrices
        2. predict: X = F*X + G*U + W  <- Get values of U 
                    P = F*P*F.transpose() + Q
        3. update:  K = P*H.transpose()*(H*P*H.transpose() + R).inverse()  (Kalman gain)
                    X = X + K*(Z - H*X)  <- Get values of Z
                    P = (I - K*H)*P(I - K*H).transpose() + K*R*K.transpose()
*/

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <ArduinoEigenDense.h>


#define SERIAL_PORT Serial

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter(uint8_t nX, uint8_t nZ, uint8_t nU, double dt, double std_U, double std_Z);
    ~KalmanFilter() {
        // Destructor
    }
    void initialize() __attribute__((weak)); //weak function to be overriden by user
    void predict(MatrixXd U_);
    void update(MatrixXd Z_);
    MatrixXd getState();
    
private:
    uint8_t nX;     //number of states
    uint8_t nZ;     //number of measurements
    uint8_t nU;     //number of control inputs
    double dt;      //time step
    double std_U;   //standard deviation of control input
    double std_Z;   //standard deviation of measurement
    MatrixXd X;     //state vector
    MatrixXd Z;     //measurement vector
    MatrixXd U;     //control vector
    MatrixXd F;     //state transition matrix
    MatrixXd G;     //control matrix
    MatrixXd P;     //state covariance matrix
    MatrixXd Q;     //process noise covariance matrix
    MatrixXd R;     //measurement noise covariance matrix
    MatrixXd H;     //measurement matrix
    MatrixXd K;     //Kalman gain matrix
    MatrixXd I;     //identity matrix
    MatrixXd W;     //process noise matrix
}; //KalmanFilter

#endif // KALMANFILTER_H
