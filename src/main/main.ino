/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include "sensors.h"
#include "kalmanfilter.h"

#define ACQUIRE_RATE 55 //Hz
#define DELTA_T (1.0f / ACQUIRE_RATE) //seconds
#define NUM_STATES 2
#define NUM_MEASUREMENTS 1
#define NUM_CONTROL_INPUTS 1

Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;
KalmanFilter myKalmanFilter_inst(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS, DELTA_T);

//redeclare the kalman filter initialize function
// void KalmanFilter::initialize(){
//     // Initialize the state transition matrix
//     F << 1, dt,
//          0, 1;

//     // Initialize the control matrix
//     G << 0.5*dt*dt,
//          dt;

//     // Initialize the process noise covariance matrix
//     Q << 0.5*dt*dt, 0,
//          0, 1;

//     // Initialize the measurement matrix
//     H << 1, 0;

//     // Initialize the measurement noise covariance matrix
//     R << 1;

//     // Initialize the process noise matrix
//     W << 0.5*dt*dt,
//          dt;

//     // Initialize the state covariance matrix
//     P << 1, 0,
//          0, 1;

//     // Initialize the state vector
//     X << 0,
//          0;
// }

//for counting loop rate
int i = 0;
unsigned long timeStart = 0;

void PrintSensorData();
void DoKalman();
void DoCount();

void setup()
{
  SERIAL_PORT.begin(921600); // Start the serial console
  
  // Initialize the Sensors
  if(mySensor_inst.init() != Sensors::SENSORS_OK){
    SERIAL_PORT.println("Sensor init failed");
    while(1);
  }

  // Calibrate the barometer
  if(mySensor_inst.CalibrateBarometerAltitude() != Sensors::SENSORS_OK){
      SERIAL_PORT.println("Barometer calibration failed");
  }

  // Calibrate the IMU
  if(mySensor_inst.CalibrateIMULinearAcceleration() != Sensors::SENSORS_OK){
      SERIAL_PORT.println("IMU calibration failed");
  }

  // Initialize the Kalman Filter
  myKalmanFilter_inst.initialize();
  delay(100);
  timeStart = millis();
}

void loop()
{
  //Read Data
  if(mySensor_inst.readData_noGPS(&sensorData_inst) != Sensors::SENSORS_OK){
    SERIAL_PORT.println("Sensor read failed");
    while(1);
  }

  DoKalman();

  PrintSensorData();

  //DoCount();
}

void DoKalman(){
  //Remove the bias from both sensors.
  double ACC_Z = sensorData_inst.imuData.LinearAccel.v2 - sensorData_inst.imuData.LinearAccelOffset.v2;  //obtained from the mean of the data
  double Alt_Baro = sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset;

  //make them into matrices
  MatrixXd Z(1,1);
  Z << Alt_Baro;
  MatrixXd U(1,1);
  U << ACC_Z*GRAVITY; //the current value is in g's, so multiply by gravity to get m/s^2

  //predict
  myKalmanFilter_inst.predict(U);

  //update
  myKalmanFilter_inst.update(Z);

  //get the state
  MatrixXd X = myKalmanFilter_inst.getState();

  //print the data
  //SERIAL_PORT.print(0x4004);
  //SERIAL_PORT.print(",");
  SERIAL_PORT.print(X(0,0));    //pos
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(X(1,0));    //vel
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(Alt_Baro);  
}

void PrintSensorData(){
  // Print the data
  //SERIAL_PORT.print(0x4008);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.barometerData.Pressure/100);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.LinearAccel.v0 - sensorData_inst.imuData.LinearAccelOffset.v0);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.LinearAccel.v1 - sensorData_inst.imuData.LinearAccelOffset.v1);
  SERIAL_PORT.print(",");
  SERIAL_PORT.println(sensorData_inst.imuData.LinearAccel.v2 - sensorData_inst.imuData.LinearAccelOffset.v2);
}

void DoCount(){
  if(i > 100){
    i = 0;
    SERIAL_PORT.printf("100 iterations done in: %d mS\n", (int)(millis()-timeStart));
    delay(5000);
    timeStart = millis();
  }
  else{
    i++;
  }
}