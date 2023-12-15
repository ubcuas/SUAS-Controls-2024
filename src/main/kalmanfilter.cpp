/*
    kalmanfilter.cpp - Implementation file for Kalman filter class
    Name: Nischay Joshi
    Date: 08-12-23
*/

#include "kalmanfilter.h"

/*
        Algortihm:

        1. initialize the vectors and matrices
        2. predict: X = F*X + G*U + W  <- Get values of U 
                    P = F*P*F.transpose() + Q
        3. update:  K = P*H.transpose()*(H*P*H.transpose() + R).inverse()  (Kalman gain)
                    X = X + K*(Z - H*X)  <- Get values of Z
                    P = (I - K*H)*P(I - K*H).transpose() + K*R*K.transpose()
*/


/*
    Constructor
    nX: number of states
    nZ: number of measurements
    nU: number of control inputs
    dt: time step
*/
KalmanFilter::KalmanFilter(uint8_t nX, uint8_t nZ, uint8_t nU, double dt) {
        // Constructor
        this->nX = nX;
        this->nZ = nZ;
        this->nU = nU;
        this->dt = dt;

        // Initialize the vectors and matrices
        X = MatrixXd::Zero(nX, 1);
        Z = MatrixXd::Zero(nZ, 1);
        U = MatrixXd::Zero(nU, 1);
        F = MatrixXd::Zero(nX, nX);
        G = MatrixXd::Zero(nX, nU);
        P = MatrixXd::Zero(nX, nX);
        Q = MatrixXd::Zero(nX, nX);
        R = MatrixXd::Zero(nZ, nZ);
        H = MatrixXd::Zero(nZ, nX);
        K = MatrixXd::Zero(nX, nZ);
        I = MatrixXd::Identity(nX, nX);
        W = MatrixXd::Zero(nX, 1);
}

/*
    Initialize Function.
    This function *CAN* overriden by the user.
    Assuming a 2 state system with 1 measurement and 1 control input
*/
void KalmanFilter::initialize(){
    // Initialize the state transition matrix
    F << 1, dt,
         0, 1;

    // Initialize the control matrix
    G << 0.5*dt*dt,
         dt;

    // Initialize the process noise covariance matrix
    Q << 0.25*dt*dt*dt*dt, 0.5*dt*dt*dt,
         0.5*dt*dt*dt, dt*dt;
    //add the accelerometer stddev
    double gravity = 9.809;
    double acc_stddev = 0.0037 * gravity;
    Q = Q * acc_stddev * acc_stddev;

    // Initialize the measurement matrix
    H << 1, 0;

    // Initialize the measurement noise covariance matrix
    double baro_stddev = 1.1466;
    R << baro_stddev * baro_stddev; //altitude barmeter stddev

    // Initialize the process noise matrix
    W << 0.5*dt*dt,
         dt;

    // Initialize the state covariance matrix
    P << 0.001, 0,
         0, 0.001;

    // Initialize the state vector
    X << 0,
         0;
}

/*
    Predict Function
    U_: Control vector
*/
void KalmanFilter::predict(MatrixXd U_) {
    // Predict the state vector
    X = F*X + G*U_ /*+ W*/;

    // Predict the state covariance matrix
    P = F*P*F.transpose() + Q;

    // Update the control vector
    U = U_;
}

/*
    Update Function
    Z_: Measurement vector
*/
void KalmanFilter::update(MatrixXd Z_) {
    // Update the measurement vector
    Z = Z_;

    // Calculate the Kalman gain
    K = P*H.transpose()*(H*P*H.transpose() + R).inverse();

    // Update the state vector
    X = X + K*(Z - H*X);

    // Update the state covariance matrix
    P = (I - K*H)*P*(I - K*H).transpose() + K*R*K.transpose();
}

/*
    Get the state vector
    Returns: X
*/
MatrixXd KalmanFilter::getState() {
    return X;
}