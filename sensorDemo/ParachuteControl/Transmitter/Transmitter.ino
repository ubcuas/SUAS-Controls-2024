//Libraries:
//For IMU 

//For server communication
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include "sensors.h"

#define SERIAL_PORT Serial

//Wifi credentials : Aim is to make a WiFiStation on ESP.
const char* ssid = "ParachuteTX";
const char* password = "UBC_UAS_2023";

typedef struct{
  double Servo0;
  double Servo1;
} RemoteData;

Sensors::sensors mySensor_inst;
Sensors::sensorData_t sensorData_inst;

//For the Webserver stuff
#define soft_AP 1 //if this is 1 then the ESP will be a soft AP otherwise it will be a station
WebServer server(80); //Webserver object
WebSocketsServer websocker_Server = WebSocketsServer(81); //Websocket object
bool connected = false; //flag to check if a client is connected

#define DELAY 20 //Delay between Transmitter updates in ms

//This function is called when a client connects to the websocket
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length){
  //Serial.printf("WebSocket %d received event: %d\n", num, type);
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket %d disconnected\n", num);
      connected = false;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = websocker_Server.remoteIP(num);
        Serial.printf("WebSocket %d connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        connected = true;
      }
      break;
    case WStype_TEXT:
      Serial.printf("WebSocket %d received text: %s\n", num, payload);
      // send message to client
      break;
    case WStype_BIN:
      Serial.printf("WebSocket %d received binary length: %u\n", num, length);
      // send message to client
      websocker_Server.sendBIN(num, payload, length);
      break;
    case WStype_ERROR:
      Serial.printf("WebSocket %d error\n", num);
      break;
    case WStype_FRAGMENT:
      Serial.printf("WebSocket %d fragment\n", num);
      break;
    default:
      Serial.printf("WebSocket %d unknown type\n", num);
      break;
  }
}

//Setup function
void setup(void) {
  //start serial communication
  Serial.begin(921600);
  while (!Serial)
    delay(10);
  
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

  //connect to WiFi, check which connection type is wanted
    //Setup soft AP
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("\nIP address: ");
    Serial.println(myIP);
    delay(5000);

  // Start the server, when new client connects, we send the index.html page.
  server.begin(); //intialize the server
  websocker_Server.begin();  // intialze the websocket server
  websocker_Server.onEvent(webSocketEvent);  // on a WS event, call webSocketEvent
  Serial.println("Setup Done");
  delay(2500);
}

void loop() {

    // Handle all the server BS
    server.handleClient(); // Listen for incoming clients
    websocker_Server.loop(); // Listen for incoming websocket clients

    //Update the IMU data
    if(mySensor_inst.readData_noGPS(&sensorData_inst) == Sensors::SENSORS_OK){
        //Send the data
        SendData();
    }
  delay(DELAY); //delay for a bit
}


/*
* Send the IMU Data through the websocket
*/
void SendData(){
    if(!connected){
        return;
    }

    // static RemoteData message = {-1000.0, -1000.0};
    // message.Servo0 += 10;
    // message.Servo1 += 10;

    // if(message.Servo0 > 1000){
    //     message.Servo0 = -1000;
    // }
    // if(message.Servo1 > 1000){
    //     message.Servo1 = -1000;
    // }

    RemoteData message;
    double pitch = sensorData_inst.imuData.EulerAngles.v1;
    //convert that to a value between -1000 and 1000
    message.Servo0 = map(pitch, -60, 60, -1000, 1000);
    message.Servo1 = message.Servo0;

    Serial.printf("Sending data: Servo0: %lf, Servo1: %lf\n", message.Servo0, message.Servo1);

    websocker_Server.broadcastBIN((uint8_t*)&message, sizeof(message), false);
}
