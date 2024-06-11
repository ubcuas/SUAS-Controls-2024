/*
  Sender.ino - Use ESP-NOW to send data from one ESP32 to another
  Created by Tayyib Chohan, 2024-03-02
  Modified by Amy Li
  
  adapted from https://dronebotworkshop.com/
*/
 
// Include Libraries
#include "Sender.h"

// Reading From Serial
bool initialized = false;
int incomingByte = 0;
char *serialVals[3]; 
char *ptr = NULL;


struct_response response;
bool success = false;
 
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

/**
 * @brief broadcast message to all recievers and recieve a response
 * 
 * @param myData
 * @return true if message was recieved by reciever successfully
 */
bool broadcastMessage(struct_message myData){
  struct_response response_2;
  success = false;
  //timer for timeout
  unsigned long start = millis();
  esp_err_t result2 = esp_now_send(0, (uint8_t *) &myData, sizeof(myData)); // peer_addr 0 means send to all
  if (result2 == ESP_OK) { 

    return true; // I AM BYPASSING RETURN MESSAGE LOL

    //Wait for confirmation from reciever
    while(!success){
      if (millis() - start > TIMEOUT_MILLIS){
        Serial.println("failed_timout");// Sending message failed timeout
        return false;
      }
    }
    if (success && response_2.bottleID == myData.bottleID){
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
  digitalWrite(LED_BLUE, HIGH);
  struct_message myData;
  int index = 0;
  while (PiSerial.available() == 0) {} // This is a blocking function
  String teststr = PiSerial.readString();
  teststr.trim();  // remove whitespace

  // convert string to char array
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
  float heading = atof(serialVals[2]);
  int bottleID = atoi(serialVals[3]);

  myData.lat = lat;
  myData.lon = lon;
  myData.heading = heading;
  myData.bottleID = bottleID;
  digitalWrite(LED_BLUE, LOW);
  return myData;
}

/**
 * @brief Handshake with reciever
 * 
 * @return true if handshake is successful 
 * @return false if handshake is unsuccessful
 */
bool handShakeSerial(){
  while (PiSerial.available() == 0) {} 
  String teststr = PiSerial.readString();
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
