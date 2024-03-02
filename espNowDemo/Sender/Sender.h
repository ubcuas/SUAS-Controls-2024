#ifndef RECIEVER_H
#define RECIEVER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>


#define DEBUG 1
#define TIMEOUT_MILLIS 5000
#define TIMEOUT_ERROR 2

// Reading From Serial
bool initialized = false;
int incomingByte = 0;
char *serialVals[3]; 
char *ptr = NULL;



// MAC Address of responder - edit as required
uint8_t broadcastAddressAll[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 
// Define a sending structure
typedef struct struct_message {
  float lat;
  float lon;
  int bottleID;
} struct_message;
 
// Define response structure
typedef struct struct_response {
  int bottleID;
} struct_response;
bool success = false;


esp_now_peer_info_t peerInfo;

struct_response response;




#endif