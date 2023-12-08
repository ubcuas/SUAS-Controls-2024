/*
    main.ino - Main File to read sensor Data using the Sensors class
    Name: Nischay Joshi
    Date: 07-12-23
*/

#include <Arduino.h>
#include "sensors.h"

Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;

int i = 0;
unsigned long timeStart = 0;

void setup()
{
  SERIAL_PORT.begin(921600); // Start the serial console
  
  // Initialize the Sensors
  if(mySensor_inst.init() != Sensors::SENSORS_OK){
    SERIAL_PORT.println("Sensor init failed");
    while(1);
  }
  delay(100);
  timeStart = millis();
}

void loop()
{
  //Read Data @ 5Hz
  if(mySensor_inst.readData_noGPS(&sensorData_inst) != Sensors::SENSORS_OK){
    SERIAL_PORT.println("Sensor read failed");
    while(1);
  }

  // Print the data
  SERIAL_PORT.print("Altitude(m): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Altitude);
  SERIAL_PORT.print("Temperature(C): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Temperature);
  SERIAL_PORT.print("Pressure(hPa): ");
  SERIAL_PORT.println(sensorData_inst.barometerData.Pressure);

  // if(i > 100){
  //   i = 0;
  //   SERIAL_PORT.printf("100 iterations done in: %d mS\n", (int)(millis()-timeStart));
  //   delay(5000);
  //   timeStart = millis();
  // }
  // else{
  //   i++;
  // }
  
  //delay(1);
}
