/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include "sensors.h"
#include "kalmanfilter.h"
#include "WebStreamServer.h"

#define ACQUIRE_RATE 57.0 //Hz
#define DELTA_T (1.0f / ACQUIRE_RATE) //seconds
#define NUM_STATES 2
#define NUM_MEASUREMENTS 1
#define NUM_CONTROL_INPUTS 1

//Sensor Noise Parameters
#define ACC_X_STD 0.03 * GRAVITY
#define ACC_Y_STD 0.03 * GRAVITY
#define ACC_Z_STD 0.05 * GRAVITY
#define BARO_ALT_STD 1.466 //meters
#define GPS_POS_STD 2.5 //meters

Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;
KalmanFilter myKalmanFilter_inst(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS, DELTA_T, ACC_Z_STD, BARO_ALT_STD);
WebStreamServer webStreamServer_inst;

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
uint8_t GPS_Loop_Counter = 0;

void PrintSensorData();
void DoKalman();
void DoCount();

void setup()
{
  SERIAL_PORT.begin(921600); // Start the serial console
  webStreamServer_inst.init();
  
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
  if(GPS_Loop_Counter >= ACQUIRE_RATE/GPS_RATE){
    if(mySensor_inst.readData_GPS(&sensorData_inst) != Sensors::SENSORS_OK){
      //SERIAL_PORT.println("Sensor read failed");
      //while(1);
    }
    GPS_Loop_Counter = 0;
  }
  else{
    if(mySensor_inst.readData_noGPS(&sensorData_inst) != Sensors::SENSORS_OK){
      //SERIAL_PORT.println("Sensor read failed");
      //while(1);
    }
    GPS_Loop_Counter++;
  }

  DoKalman();

  PrintSensorData();

  //DoCount();
}

void DoKalman(){
  //Remove the bias from both sensors.
  double ACC_Z = sensorData_inst.imuData.LinearAccel.v2 /*- sensorData_inst.imuData.LinearAccelOffset.v2*/;  //obtained from the mean of the data
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

  //incase X is not a number reset the filter
  if(isnan(myKalmanFilter_inst.getState()(0,0)) || isnan(myKalmanFilter_inst.getState()(1,0))){
    //reset the filter
    myKalmanFilter_inst.initialize();
  }

  //print the data
  //SERIAL_PORT.print(0x4004);
  //SERIAL_PORT.print(",");
}

void PrintSensorData(){

  char buffer[500];
  // Print the data
  //SERIAL_PORT.print(0x4008);
  //get the state
  MatrixXd X = myKalmanFilter_inst.getState();
  snprintf(buffer, sizeof(buffer), "%.2lf,%.2lf,%.2lf,%.3lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.2lf,%d,%d,%.3lf,%.3lf,%.3lf,%.3lf\n", 
    X(0,0),    //Kalman-pos
    X(1,0),    //Kalman-vel
    sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset,
    sensorData_inst.barometerData.Pressure/100,
    sensorData_inst.imuData.LinearAccel.v0 /*- sensorData_inst.imuData.LinearAccelOffset.v0*/, 
    sensorData_inst.imuData.LinearAccel.v1 /*- sensorData_inst.imuData.LinearAccelOffset.v1*/, 
    sensorData_inst.imuData.LinearAccel.v2 /*- sensorData_inst.imuData.LinearAccelOffset.v2*/, 
    sensorData_inst.gpsData.Latitude, 
    sensorData_inst.gpsData.Longitude, 
    sensorData_inst.gpsData.Altitude, 
    sensorData_inst.gpsData.lock? 1 : 0, 
    sensorData_inst.gpsData.satellites, 
    sensorData_inst.imuData.Orientation.q0, 
    sensorData_inst.imuData.Orientation.q1, 
    sensorData_inst.imuData.Orientation.q2, 
    sensorData_inst.imuData.Orientation.q3);
  // Send the data to all connected WebSocket clients
  if(webStreamServer_inst.send(buffer) == WebStreamServer::SUCCESS){
    //SERIAL_PORT.println(mycounter);
    //mycounter++;
  }
  SERIAL_PORT.print(buffer);
}

void DoCount(){
  if(i > 1000){
    i = 0;
    SERIAL_PORT.printf("100 iterations done in: %d mS\n", (int)(millis()-timeStart));
    delay(5000);
    timeStart = millis();
  }
  else{
    i++;
  }
}

void OLD_PRINT(){
  MatrixXd X = myKalmanFilter_inst.getState();
  SERIAL_PORT.print(X(0,0));    //Kalman-pos
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(X(1,0));    //Kalman-vel
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.barometerData.Pressure/100, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.LinearAccel.v0 /*- sensorData_inst.imuData.LinearAccelOffset.v0*/, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.LinearAccel.v1 /*- sensorData_inst.imuData.LinearAccelOffset.v1*/, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.LinearAccel.v2 /*- sensorData_inst.imuData.LinearAccelOffset.v2*/, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.gpsData.Latitude, 10);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.gpsData.Longitude, 10);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.gpsData.Altitude, 6);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.gpsData.lock? "LOCKED" : "NOT LOCKED");
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.gpsData.satellites);
  // SERIAL_PORT.print(0x1001);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.Orientation.q0);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.Orientation.q1);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(sensorData_inst.imuData.Orientation.q2);
  SERIAL_PORT.print(",");
  SERIAL_PORT.println(sensorData_inst.imuData.Orientation.q3);
}