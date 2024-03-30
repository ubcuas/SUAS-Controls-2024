/*
  Sender.ino - Use ESP-NOW to send data from one ESP32 to another
  Created by Tayyib Chohan, 2024-03-02
  
  adapted from https://dronebotworkshop.com/
*/
 
// Include Libraries
#include "Sender.h"
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(DEBUG){
    Serial.print("Last Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");    
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&response, (struct_response*)incomingData, sizeof(response));
  success = true;
  if (DEBUG){
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("bottleID: ");
    Serial.println(response.bottleID);
    Serial.println();
  }
}
 
void setup() {
  
  // Set up Serial Monitor
  Serial.begin(115200);
 
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register the recive callback
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  memcpy(peerInfo.peer_addr, broadcastAddressAll, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }        

}


void loop() {
  // Wait for Software to Initialize the Handshake
  while(!initialized){
    initialized = handShakeSerial();
  }
  
  struct_message myData = recieveData();
  broadcastMessage(myData);

  delay(2000);
}

/**
 * @brief broadcast message to all recievers and recieve a response
 * 
 * @param myData
 * @return true if message was recieved by reciever successfully
 */
bool broadcastMessage(struct_message myData){
  struct_response response;
  success = false;
  //timer for timeout
  unsigned long start = millis();
  esp_err_t result2 = esp_now_send(broadcastAddressAll, (uint8_t *) &myData, sizeof(myData));
  if (result2 == ESP_OK) {  
    //Wait for confirmation from reciever
    while(!success){
      if (millis() - start > TIMEOUT_MILLIS){
        Serial.println("failed_timout");// Sending message failed timeout
        return false;
      }
    }
    if (success && response.bottleID == bottleID){
      Serial.println("success");// Message received by reciever
      return true;
    }
    else {
      Serial.println("failed_wrongBottle");// Sending message failed revieved wrong bottleID
      return false;
    }
  }
  else {
    Serial.println("failed_unknownfailure");// Sending message failed
    return false;
  }
  Serial.println("failed_toSend");// Sending message failed
  return false;
}


/**
 * @brief Recieve data from Serial and convert to struct_message
 *  This function will block until data is recieved
 * @return struct_message 
*/
struct_message recieveData(){
  struct_message myData;  
  int index = 0;
  while (Serial.available() == 0) {} // This is a blocking function
  String teststr = Serial.readString();
  teststr.trim();  // remove whitespace

  //convert string to char array
  char charBuf[teststr.length()+1];
  teststr.toCharArray(charBuf, teststr.length()+1);

   ptr = strtok(charBuf, ",");
   while (ptr != NULL)
   {
      serialVals[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
   }
  
  float lat = atof(serialVals[0]);
  float lon = atof(serialVals[1]);
  int bottleID = atoi(serialVals[2]);

  myData.lat = lat;
  myData.lon = lon;
  myData.bottleID = bottleID;
  return myData;
}

/**
 * @brief Handshake with reciever
 * 
 * @return true if handshake is successful 
 * @return false if handshake is unsuccessful
 */
bool handShakeSerial(){
  while (Serial.available() == 0) {} 
  String teststr = Serial.readString();
  teststr.trim();  // remove whitespace
  if (teststr == "UBC"){
    Serial.println("UAS");
    return true;
  }
  else{
    Serial.println("Invalid Handshake");
  }
  return false;
}
