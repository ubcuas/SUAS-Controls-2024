#ifndef RECIEVER_H
#define RECIEVER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG 1
#define BOTTLE_ID 1

// Structure example to receive data
// Must match the sender structure
typedef struct datapacket {
  float lat;
  float lon;
  float heading;
  int bottleID;
} datapacket;

typedef struct responsepacket {
  int bottleID;
} responsepacket;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
bool InitESPNow(uint8_t Bottle_id);

#endif