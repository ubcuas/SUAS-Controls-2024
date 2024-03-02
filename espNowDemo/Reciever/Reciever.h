#ifndef RECIEVER_H
#define RECIEVER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG 1
#define BOTTLE_ID 1

//Structure example to receive data
//Must match the sender structure
typedef struct datapacket {
  float lat;
  float lon;
  int bottleID;
} datapacket;

typedef struct responsepacket {
  int bottleID;
} responsepacket;
//Create a struct_message called myData
datapacket myData;
responsepacket response;



#endif