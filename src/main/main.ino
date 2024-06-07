/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "ConfigParse.h"
#include "sensors.h"
#include "kalmanfilter.h"
#include "WebStreamServer.h"
#include "SDCard.h"
#include "encoder_steering.h"
#include "Reciever.h"

#define BufferLen 500

// Kalman Filter Parameters
#define NUM_STATES 2
#define NUM_MEASUREMENTS 1
#define NUM_CONTROL_INPUTS 1

//config data structure and object
ConfigParser configParser_inst;
ConfigData_t configData_inst;

TinyGPSPlus GPS;
Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;
KalmanFilter myKalmanFilter_inst_Z(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS);
KalmanFilter myKalmanFilter_inst_Y(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS);
KalmanFilter myKalmanFilter_inst_X(NUM_STATES, NUM_MEASUREMENTS, NUM_CONTROL_INPUTS);
WebStreamServer webStreamServer_inst;

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
  bool SDStatus = true;
  SERIAL_PORT.begin(921600); // Start the serial console
  
  if(SDCard::SDcardInit() != SDCard::SDCARD_OK){
    SERIAL_PORT.println("SD card failed");
    SDStatus = false;
  }

  // Read the configuration file
  if(SDStatus == 1){
    if(configParser_inst.parseConfigFile(SD, &configData_inst) != ConfigParseStatus::CONFIG_OK){
      SERIAL_PORT.println("Config read failed");
      SERIAL_PORT.println("Using default values");
      configParser_inst.getConfigNoSD(&configData_inst);
    }
  }
  else{
    configParser_inst.getConfigNoSD(&configData_inst);
  }
  // Print the configuration data
  configParser_inst.printConfigData(&configData_inst);

  // Initialize the WebStreamServer
  webStreamServer_inst.init((const char *)configData_inst.SSID, (const char *)configData_inst.Password);
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
  myKalmanFilter_inst_Z.initialize(configData_inst.SampleTime, configData_inst.ACC_Z_STD, configData_inst.BARO_ALT_STD);
  myKalmanFilter_inst_Y.initialize(configData_inst.SampleTime, configData_inst.ACC_Y_STD, configData_inst.GPS_POS_STD);
  myKalmanFilter_inst_X.initialize(configData_inst.SampleTime, configData_inst.ACC_X_STD, configData_inst.GPS_POS_STD);
  
  //Steering setup
  steering_setup(configData_inst.AcquireRate, configData_inst.PID.KP, configData_inst.PID.KI, configData_inst.PID.KD);
  
  // Set up ESP-NOW communication
  if(InitESPNow(configData_inst.BottleID) == false){
    SERIAL_PORT.println("ESP-NOW init failed");
  }

  delay(1000);
  timeStart = millis();
}

void loop()
{
  //Read Data
  if(GPS_Loop_Counter >= configData_inst.AcquireRate/GPS_RATE){
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
  ComputePID();

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
  ACC_Z = ACC_Z * configData_inst.GRAVITY;  //the current value is in g's, so multiply by gravity to get m/s^2
  double Alt_Baro = sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset;
  //--Y_AXIS_KALMAN--
  double ACC_Y = sensorData_inst.imuData.LinearAccel.v1 - sensorData_inst.imuData.LinearAccelOffset.v1;  //obtained from the mean of the data
  ACC_Y = ACC_Y * configData_inst.GRAVITY;
  double Y_POS_GPS = sensorData_inst.gpsData.Ypos;
  //--X_AXIS_KALMAN--
  double ACC_X = sensorData_inst.imuData.LinearAccel.v0 - sensorData_inst.imuData.LinearAccelOffset.v0;  //obtained from the mean of the data
  ACC_X = ACC_X * configData_inst.GRAVITY;
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
  char *buffer;
  char *buffer_wifi; 
  
  //allocate memory for the buffer
  buffer = (char *)malloc(configData_inst.BufferSize * sizeof(char));
  buffer_wifi = (char *)malloc(configData_inst.BufferSize * sizeof(char));

  //if null pointers then return
  if(buffer == NULL || buffer_wifi == NULL){
    return;
  }

  static uint32_t time = millis();
  // Print the data
  //get the state
  MatrixXd X_Zaxis = myKalmanFilter_inst_Z.getState();
  MatrixXd X_Yaxis = myKalmanFilter_inst_Y.getState();
  MatrixXd X_Xaxis = myKalmanFilter_inst_X.getState();

  //prepare the Serial and SD card buffer based on which data is to be printed
  static bool headerWritten = false;
  uint16_t length = 0;

  if (!headerWritten) {
    // Construct the CSV header based on config settings
    length += snprintf(buffer + length, configData_inst.BufferSize - length, "hyu");

    if(configData_inst.OutputData.KalmanFilter.Position) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",PosX,PosY,PosZ");
    }
    if(configData_inst.OutputData.KalmanFilter.Velocity) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",VelX,VelY,VelZ");
    }
    if(configData_inst.OutputData.BAROMETER.Altitude) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",BaroAltitude");
    }
    if(configData_inst.OutputData.BAROMETER.Pressure) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",BaroPressure");
    }
    if(configData_inst.OutputData.IMU.LinearAccel) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",AccelX,AccelY,AccelZ");
    }
    if(configData_inst.OutputData.IMU.Orientation_Quaternion) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",OrientQ0,OrientQ1,OrientQ2,OrientQ3");
    }
    if(configData_inst.OutputData.IMU.EulerAngles) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",Roll,Pitch,Yaw");
    }
    if(configData_inst.OutputData.GPS.Coordinates) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",Latitude,Longitude");
    }
    if(configData_inst.OutputData.GPS.Altitude) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",GPSAltitude");
    }
    if(configData_inst.OutputData.GPS.Satellites) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",Satellites");
    }
    if(configData_inst.OutputData.GPS.Lock) {
        length += snprintf(buffer + length, configData_inst.BufferSize - length, ",GPSLock");
    }

    // Append a new line after the header
    length += snprintf(buffer + length, configData_inst.BufferSize - length, "\n");

    // Write the header to the SD card or other output destinations here
    SDCard::SDCardStatus SDWriteStatus = SDCard::SDcardWrite(buffer);
    if(SDWriteStatus == SDCard::SDCARD_ERROR){ //only alert if the writing failed for some reason.
      Serial.println("SD card write failed");
    } 

    headerWritten = true; // Set the flag so the header is not written again

    //reset the length to 0
    length = 0;
  }


  //Add header to the buffer
  length += snprintf(buffer + length, configData_inst.BufferSize, "hyu");

  //Kalman Filter Data
  //   -- Position
  if(configData_inst.OutputData.KalmanFilter.Position){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.2lf,%.2lf,%.2lf", X_Xaxis(0,0), X_Yaxis(0,0), X_Zaxis(0,0));
  }
  //   -- Velocity
  if(configData_inst.OutputData.KalmanFilter.Velocity){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.2lf,%.2lf,%.2lf", X_Xaxis(1,0), X_Yaxis(1,0), X_Zaxis(1,0));
  }

  //Barometer Data
  if(configData_inst.OutputData.BAROMETER.Altitude){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.2lf", sensorData_inst.barometerData.Altitude - sensorData_inst.barometerData.AltitudeOffset);
  }
  if(configData_inst.OutputData.BAROMETER.Pressure){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.3lf", sensorData_inst.barometerData.Pressure/100.0);
  }

  //IMU Data
  if(configData_inst.OutputData.IMU.LinearAccel){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.6lf,%.6lf,%.6lf", sensorData_inst.imuData.LinearAccel.v0 - sensorData_inst.imuData.LinearAccelOffset.v0, sensorData_inst.imuData.LinearAccel.v1 - sensorData_inst.imuData.LinearAccelOffset.v1, sensorData_inst.imuData.LinearAccel.v2 - sensorData_inst.imuData.LinearAccelOffset.v2);
  }
  if(configData_inst.OutputData.IMU.Orientation_Quaternion){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.6lf,%.6lf,%.6lf,%.6lf", sensorData_inst.imuData.Orientation.q0, sensorData_inst.imuData.Orientation.q1, sensorData_inst.imuData.Orientation.q2, sensorData_inst.imuData.Orientation.q3);
  }
  if(configData_inst.OutputData.IMU.EulerAngles){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.3lf,%.3lf,%.3lf", sensorData_inst.imuData.EulerAngles.v0, sensorData_inst.imuData.EulerAngles.v1, sensorData_inst.imuData.EulerAngles.v2);
  }

  //GPS Data
  if(configData_inst.OutputData.GPS.Coordinates){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.6lf,%.6lf", sensorData_inst.gpsData.Latitude, sensorData_inst.gpsData.Longitude);
  }
  if(configData_inst.OutputData.GPS.Altitude){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%.2lf", sensorData_inst.gpsData.Altitude);
  }
  if(configData_inst.OutputData.GPS.Satellites){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%d", sensorData_inst.gpsData.satellites);
  }
  if(configData_inst.OutputData.GPS.Lock){
    length += snprintf(buffer + length, configData_inst.BufferSize, ",%d", sensorData_inst.gpsData.lock? 1 : 0);
  }

  //Add a new line character
  length += snprintf(buffer + length, configData_inst.BufferSize, "\n");

  // Write the Data
  SDCard::SDCardStatus SDWriteStatus = SDCard::SDcardWrite(buffer);
  if(SDWriteStatus == SDCard::SDCARD_ERROR){ //only alert if the writing failed for some reason.
    Serial.println("SD card write failed");
  } 
  if(configData_inst.Print_Enable){
    SERIAL_PORT.print(buffer);
  }
  
  snprintf(buffer_wifi, configData_inst.BufferSize, "hyu, %.2lf,%.2lf,%.2lf,%.6lf,%.6lf,%.2lf,%d,%d\n", //hyu, %.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.6lf,%.6lf,%.2lf,%d,%d\n
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
    if(webStreamServer_inst.send(buffer_wifi) == WebStreamServer::SUCCESS){
    //SERIAL_PORT.println(mycounter);
    //mycounter++;
    }
  }
  free(buffer);
  free(buffer_wifi);
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

void ComputePID(){
  char outputBuffer[BufferLen]; //This is for printing values out to terminal
  double target_lon=-122.12071003376428; //Example target longitude
  double target_lat=37.4181048968111; //Example target latitude

  //To access kalman filter values for current x,y,&z direction
  MatrixXd X_Zaxis = myKalmanFilter_inst_Z.getState();
  MatrixXd X_Yaxis = myKalmanFilter_inst_Y.getState();
  MatrixXd X_Xaxis = myKalmanFilter_inst_X.getState();

  // Take current GPS coordinates and add x,y,z, from kalman filter
  // Lat_Fast = GPS.Fast+X_Moved*ConversionFactor
  // radius_calc= 180*radius of earth/pi
  double radius_calc = 365285454.545;

  double rEarth_m = 6371000.0;
  double lat_dist_per_degree_m = 111000.0;

  double delta_lat = X_Yaxis(0,0)/lat_dist_per_degree_m;
  double delta_long = X_Xaxis(0,0)/(lat_dist_per_degree_m * cos(sensorData_inst.gpsData.refLatitude * M_PI/180.0));

  double lon_now = sensorData_inst.gpsData.refLongitude + delta_long;
  double lat_now = sensorData_inst.gpsData.refLatitude + delta_lat;
  //Height = current GPS_Position & use kinematics to get new height? To be done
  
  //Get process variable - pv is the error between (lot & lat_fast direction)-(target) / current heading-target heading
  double setpoint = GPS.courseTo(lat_now, lon_now, target_lat, target_lon); //Get the heading to the target
  //convert heading from 0-360 to -180-180
  if(setpoint > 180){
    setpoint = setpoint - 360;
  }

  // make the Data packet
  AngleData data = {setpoint, sensorData_inst.imuData.EulerAngles.v2, 0};
  // send the data packet to the queue
  if (sensorData_inst.barometerData.Altitude <= configData_inst.HEIGHT_THRESH){
    sendSteeringData(data);
  }
  else {
    count_1 = 0; // Keep resetting encoders to zero until steering activated
    count_2 = 0;
  }
  

  double distance = GPS.distanceBetween(lon_now, lat_now, target_lon, target_lat);
  
  //Print to terminal
  snprintf(outputBuffer, BufferLen, "Yaw: %.3lf\nSetpoint: %.3lf\nDistance: %.3lf\nlattitude: %.6lf\nlongitude: %.6lf\nlat_now: %.6lf\nlon_now: %.6lf\n,x: %.6lf\ny: %.6lf\nz: %.6lf\n", 
  sensorData_inst.imuData.EulerAngles.v2, 
  setpoint, 
  distance,
  sensorData_inst.gpsData.Latitude,
  sensorData_inst.gpsData.Longitude,
  lat_now,
  lon_now,
  X_Xaxis(0,0),
  X_Yaxis(0,0),
  X_Zaxis(0,0)
  );
  SERIAL_PORT.print(outputBuffer);
}

