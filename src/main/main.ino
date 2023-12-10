/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include "sensors.h"
#include "kalmanfilter.h"

#define ACQUIRE_RATE 58 //Hz
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


int i = 0;
unsigned long timeStart = 0;

void PrintSensorData();

void setup()
{
  SERIAL_PORT.begin(921600); // Start the serial console
  
  // Initialize the Sensors
  if(mySensor_inst.init() != Sensors::SENSORS_OK){
    SERIAL_PORT.println("Sensor init failed");
    while(1);
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

  //Remove the bias from both sensors.
  double ACC_Z = sensorData_inst.imuData.LinearAccel.v2 -= 0.034637924;  //obtained from the mean of the data
  double Alt_Baro = sensorData_inst.barometerData.Altitude -= 202.2801934; //obtained from the mean of the data

  //make them into matrices
  MatrixXd Z(1,1);
  Z << Alt_Baro;
  MatrixXd U(1,1);
  U << ACC_Z;

  //predict
  myKalmanFilter_inst.predict(U);

  //update
  myKalmanFilter_inst.update(Z);

  //get the state
  MatrixXd X = myKalmanFilter_inst.getState();

  //print the data
  // SERIAL_PORT.print("Altitude(m): ");
  SERIAL_PORT.print(X(0,0));
  SERIAL_PORT.print(",");
  // SERIAL_PORT.print("Velocity(m/s): ");
  SERIAL_PORT.println(X(1,0));
  
}


void PrintSensorData(){
// Print the data
  SERIAL_PORT.print("Altitude(m): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Altitude);
  SERIAL_PORT.print("Acceleration(m/s^2): ");
  SERIAL_PORT.println(sensorData_inst.imuData.LinearAccel.v2);
  SERIAL_PORT.print("Temperature(C): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Temperature);
  SERIAL_PORT.print("Pressure(hPa): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Pressure);
  if(i > 100){
    i = 0;
    SERIAL_PORT.printf("100 iterations done in: %d mS\n", (int)(millis()-timeStart));
    delay(5000);
    timeStart = millis();
  }
  else{
    i++;
  }
  //delay(1);
}