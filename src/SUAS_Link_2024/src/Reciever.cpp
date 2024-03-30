/*
  Reciever.ino - ESP32 reciever code for the ESP-NOW communication protocol
  Created by Tayyib Chohan, 2024-03-02
  
  adapted from https://dronebotworkshop.com/
*/

#include "Reciever.h"

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, (datapacket*)incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("bottleID: ");
  Serial.println(myData.bottleID);
  Serial.print("lat: ");
  Serial.println(myData.lat);
  Serial.print("lon: ");
  Serial.println(myData.lon);
  Serial.println();

  // Send a response
  if (myData.bottleID == BOTTLE_ID){
    response.bottleID = BOTTLE_ID;
    esp_err_t result = esp_now_send(mac, (uint8_t *) &response, sizeof(response));
    if (result == ESP_OK) {
      Serial.println("Response sent");
    }
    else {
      Serial.println("Error sending response");
    }
  }
  else{
    Serial.println("Not my bottle");
  }
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  //Register peer
  esp_now_peer_info_t peerInfo;

}
 
void loop() {

}