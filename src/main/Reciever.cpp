/*
  Reciever.ino - ESP32 reciever code for the ESP-NOW communication protocol
  Created by Tayyib Chohan, 2024-03-02
  Modified by Amy Li
  
  adapted from https://dronebotworkshop.com/
*/
#include "Reciever.h"

datapacket myData;
responsepacket response;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, (datapacket*)incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("lat: ");
  Serial.println(myData.lat);
  Serial.print("lon: ");
  Serial.println(myData.lon);
  Serial.print("heading: ");
  Serial.println(myData.heading);
  Serial.print("bottleID: ");
  Serial.println(myData.bottleID);
  Serial.println();

  // Send a response
  if (myData.bottleID == BOTTLE_ID){
    response.bottleID = BOTTLE_ID;
    esp_err_t result = esp_now_send(mac, (uint8_t *) &response, sizeof(response));
    if (result == ESP_OK) {
      Serial.println("Response sent");
    }
    else {
      Serial.println("Error sending response"); // Currently not working, might have to add peer?
    }
  }
  else{
    Serial.println("Not my bottle");
  }
}
