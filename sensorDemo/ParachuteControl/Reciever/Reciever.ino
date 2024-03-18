#include <Arduino.h>
#include "Steering.h"
#include "SDCard.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

#define SERIAL_PORT Serial

const char* ssid = "ParachuteTX";
const char* password = "UBC_UAS_2023";
const char* ServerIP = "192.168.4.1";
uint16_t ServerPort = 81;

WebSocketsClient webSocket_Client;   //Websocket client object

#define DELAY 20 //Delay between Reciever writes in ms

typedef struct{
  double Servo0;
  double Servo1;
} RemoteData;


//Web socket event handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_DISCONNECTED){
      Serial.printf("[WSc] Disconnected!\n");
  } else if(type == WStype_CONNECTED){
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      //Send the message to the server
      webSocket_Client.sendTXT("Hello Server");
  }  else if(type == WStype_BIN){
    // Get the data
    RemoteData* data = (RemoteData*)payload;
    // print data
    Serial.printf("Received data: Servo0: %lf, Servo1: %lf\n", data->Servo0, data->Servo1);
    //Steering function
    steering(data->Servo0);
  }
}

void setup() {
  Serial.begin(921600);

  if(SDCard::SDcardInit() != SDCard::SDCARD_OK){
    SERIAL_PORT.println("SD card failed");
  }

  //Setup WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  //Setup Websocket Client
  webSocket_Client.begin(ServerIP, ServerPort, "/"); //Connect to the server
  //Add Websocket event handler
  webSocket_Client.onEvent(webSocketEvent);
  //Retry connetction time
  webSocket_Client.setReconnectInterval(5000);
  
  motorSetup(); //Initialize two servo motors

  delay(1000);
}

void loop(){

    //Handle Websocket events
    webSocket_Client.loop();



}