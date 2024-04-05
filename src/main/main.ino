/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "sensors.h"
#include "kalmanfilter.h"
#include "WebStreamServer.h"
#include "SDCard.h"
#include "PID.h"
#include "Steering.h"
#include "Reciever.h"

#define BufferLen 150

#define ACQUIRE_RATE 57.0 //Hz
#define DELTA_T (1.0f / ACQUIRE_RATE) //seconds
#define NUM_STATES 2
#define NUM_MEASUREMENTS 1
#define NUM_CONTROL_INPUTS 1

//Sensor Noise Parameters
#define ACC_X_STD 0.3 * GRAVITY
#define ACC_Y_STD 0.3 * GRAVITY
#define ACC_Z_STD 0.05 * GRAVITY
#define BARO_ALT_STD 1.466 //meters
#define GPS_POS_STD 2.5 //meters

TinyGPSPlus GPS;
Pid PID;
Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;
KalmanFilter myKalmanFilter_inst_Z(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS, DELTA_T, ACC_Z_STD, BARO_ALT_STD);
KalmanFilter myKalmanFilter_inst_Y(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS, DELTA_T, ACC_Y_STD, GPS_POS_STD);
KalmanFilter myKalmanFilter_inst_X(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS, DELTA_T, ACC_X_STD, GPS_POS_STD);
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

// esp_now_peer_info_t peerInfo;

void PrintSensorData();
void DoKalman();
void DoCount();

void setup()
{
  SERIAL_PORT.begin(921600); // Start the serial console
  webStreamServer_inst.init();
  webStreamServer_inst.setCustomFunction([&]() { mySensor_inst.resetGPSReference(); });

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
  myKalmanFilter_inst_Z.initialize();
  myKalmanFilter_inst_Y.initialize();
  myKalmanFilter_inst_X.initialize();
  
  if(SDCard::SDcardInit() != SDCard::SDCARD_OK){
    SERIAL_PORT.println("SD card failed");
  }
  //Initializes PID object
  PID.PIDInit(ACQUIRE_RATE);
  motorSetup(); //Initialize two servo motors
  
  // Set up ESP-NOW communication
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); }
  esp_now_register_recv_cb(OnDataRecv);
  // peerInfo.channel = 0;  
  // peerInfo.encrypt = false;

  delay(1000);
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
  PIDTesting();

  //keep reading battery data
  // if(mySensor_inst.UpdateBatteryData(&sensorData_inst) != Sensors::SENSORS_OK){
  //   SERIAL_PORT.println("Battery read failed");
  // }
  // else {
  //    SERIAL_PORT.printf("Battery voltage is: %f\nVoltage Read: %f\nAnalog Read : %d\n", sensorData_inst.batteryData.LipoVoltage, sensorData_inst.batteryData.ReadVoltage, sensorData_inst.batteryData.ReadValue);
  // }
  // delay(500);
  //DoCount();
}

void DoKalman(){
  //Remove the bias from sensors.

  //---Z_AXIS_KALMAN---
  double ACC_Z = sensorData_inst.imuData.LinearAccel.v2 - sensorData_inst.imuData.LinearAccelOffset.v2;  //obtained from the mean of the data
  ACC_Z = ACC_Z * GRAVITY;  //the current value is in g's, so multiply by gravity to get m/s^2
  double Alt_Baro = sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset;
  //--Y_AXIS_KALMAN--
  double ACC_Y = sensorData_inst.imuData.LinearAccel.v1 - sensorData_inst.imuData.LinearAccelOffset.v1;  //obtained from the mean of the data
  ACC_Y = ACC_Y * GRAVITY;
  double Y_POS_GPS = sensorData_inst.gpsData.Ypos;
  //--X_AXIS_KALMAN--
  double ACC_X = sensorData_inst.imuData.LinearAccel.v0 - sensorData_inst.imuData.LinearAccelOffset.v0;  //obtained from the mean of the data
  ACC_X = ACC_X * GRAVITY;
  double X_POS_GPS = sensorData_inst.gpsData.Xpos;

  //make them into matrices
  //---Z_AXIS_KALMAN---
  MatrixXd Z_Zaxis(1,1);
  Z_Zaxis << Alt_Baro;
  MatrixXd U_Zaxis(1,1);
  U_Zaxis << ACC_Z;
  //---Y_AXIS_KALMAN---
  MatrixXd Z_Yaxis(1,1);
  Z_Yaxis << Y_POS_GPS;
  MatrixXd U_Yaxis(1,1);
  U_Yaxis << ACC_Y;
  //---X_AXIS_KALMAN---
  MatrixXd Z_Xaxis(1,1);
  Z_Xaxis << X_POS_GPS;
  MatrixXd U_Xaxis(1,1);
  U_Xaxis << ACC_X;
   
  //predict
  myKalmanFilter_inst_Z.predict(U_Zaxis);
  myKalmanFilter_inst_Y.predict(U_Yaxis);
  myKalmanFilter_inst_X.predict(U_Xaxis);

  //update
  myKalmanFilter_inst_Z.update(Z_Zaxis);
  myKalmanFilter_inst_Y.update(Z_Yaxis);
  myKalmanFilter_inst_X.update(Z_Xaxis);

}

void PrintSensorData(){

  char buffer[500];
  char buffer1[500];
  
  static uint32_t time = millis();
  // Print the data
  //get the state
  MatrixXd X_Zaxis = myKalmanFilter_inst_Z.getState();
  MatrixXd X_Yaxis = myKalmanFilter_inst_Y.getState();
  MatrixXd X_Xaxis = myKalmanFilter_inst_X.getState();
  snprintf(buffer, sizeof(buffer), "hyu, %.2lf,%.2lf,%.2lf,%.3lf,%.6lf,%.6lf,%.6lf,%.6lf,%.6lf,%.2lf,%d,%d,%.3lf,%.3lf,%.3lf,%.3lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\n", 
    X_Zaxis(0,0),    //Kalman-pos_Zaxis
    X_Zaxis(1,0),    //Kalman-vel_Zaxis
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
    sensorData_inst.imuData.Orientation.q3,
    X_Xaxis(0,0),    //Kalman-pos_Xaxis
    X_Xaxis(1,0),    //Kalman-vel_Xaxis
    X_Yaxis(0,0),    //Kalman-pos_Yaxis
    X_Yaxis(1,0),     //Kalman-vel_Yaxis
    sensorData_inst.imuData.EulerAngles.v0, //roll
    sensorData_inst.imuData.EulerAngles.v1, //pitch
    sensorData_inst.imuData.EulerAngles.v2  //yaw
  );

  snprintf(buffer1, sizeof(buffer1), "hyu, %.2lf,%.2lf,%.2lf,%.6lf,%.6lf,%.2lf,%d,%d\n", //hyu, %.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.6lf,%.6lf,%.2lf,%d,%d\n
    X_Xaxis(0,0),    //Kalman-pos_Xaxis
    X_Yaxis(0,0),    //Kalman-pos_Yaxis
    X_Zaxis(0,0),    //Kalman-pos_Zaxis
    // X_Xaxis(1,0),    //Kalman-vel_Xaxis
    // X_Yaxis(1,0),    //Kalman-vel_Yaxis
    // X_Zaxis(1,0),     //Kalman-vel_Zaxis
    sensorData_inst.gpsData.Latitude, 
    sensorData_inst.gpsData.Longitude, 
    sensorData_inst.gpsData.Altitude,
    sensorData_inst.gpsData.lock? 1 : 0, 
    sensorData_inst.gpsData.satellites
  );
  // Send the data to all connected WebSocket clients
  if(millis() - time > 120){
    time = millis();
    if(webStreamServer_inst.send(buffer1) == WebStreamServer::SUCCESS){
    //SERIAL_PORT.println(mycounter);
    //mycounter++;
  }
  }
  SDCard::SDCardStatus SDWriteStatus = SDCard::SDcardWrite(buffer);
  if(SDWriteStatus == SDCard::SDCARD_ERROR){ //only alert if the writing failed for some reason.
    Serial.println("SD card write failed");
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

void PIDTesting(){
  char outputBuffer[BufferLen]; //This is for printing values out to terminal
  float target_lon=0; //Example target longitude
  float target_lat=0; //Example target latitude

  //To access kalman filter values for current x,y,&z direction
  MatrixXd X_Zaxis = myKalmanFilter_inst_Z.getState();
  MatrixXd X_Yaxis = myKalmanFilter_inst_Y.getState();
  MatrixXd X_Xaxis = myKalmanFilter_inst_X.getState();

  // Take current GPS coordinates and add x,y,z, from kalman filter
  // Lat_Fast = GPS.Fast+X_Moved*ConversionFactor
  //radius_calc= 180*radius of earth/pi
  float radius_calc = 365438211.3124;
  float lon_fast = sensorData_inst.gpsData.Longitude + X_Xaxis(0,0)*radius_calc;
  float lat_fast = sensorData_inst.gpsData.Latitude + X_Yaxis(0,0)*radius_calc;
  //Height = current GPS_Position & use kinematics to get new height? To be done
  
  //Get process variable - pv is the error between (lot & lat_fast direction)-(target) / current heading-target heading
  double pv = GPS.courseTo(lat_fast,lon_fast,target_lat,target_lon)-sensorData_inst.imuData.EulerAngles.v2;

  //compute PID angle using process variable
  double motor_value = PID.PIDcalculate(pv);

  //Send to servos
  steering(motor_value);
  
  //Print to terminal
  //sprintf(outputBuffer, "Pv: %.5lf \t Error: %.5lf \t Output: %.5lf\t Integral: %.5lf \t \n", pv, PID.error, yaw, PID.integral); 
  SERIAL_PORT.print(outputBuffer);
}

void OLD_PRINT(){
  MatrixXd X = myKalmanFilter_inst_Z.getState();
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