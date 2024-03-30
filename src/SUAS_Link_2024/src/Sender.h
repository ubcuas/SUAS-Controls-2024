#ifndef SENDER_H
#define SENDER_H

#include <Arduino.h>
// // #include <esp_now.h>
// #include <ESPNowW.h>
// #include <WiFi.h>

// #include "params.h"

// #define DEBUG 1
// #define TIMEOUT_MILLIS 5000
// #define TIMEOUT_ERROR 2

// // Reading From Serial
// bool initialized = false;
// int incomingByte = 0;
// char *serialVals[3]; 
// char *ptr = NULL;

// // MAC Address of responder - edit as required
// uint8_t broadcastAddressAll[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 
// Define a sending structure
typedef struct struct_message {
  float lat;
  float lon;
  float heading;
  int bottleID;
} struct_message;
 
// // Define response structure
// typedef struct struct_response {
//   int bottleID;
// } struct_response;
// bool success = false;

// esp_now_peer_info_t peerInfo;
// struct_response response;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
// bool broadcastMessage(struct_message myData);
// struct_message recieveData();
// bool handShakeSerial();

#endif