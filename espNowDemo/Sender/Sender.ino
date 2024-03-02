/*
  ESP-NOW Demo - Transmit
  esp-now-demo-xmit.ino
  Sends data to Responder
  
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
 
// Include Libraries
#include <esp_now.h>
#include <WiFi.h>


#define DEBUG 1
#define TIMEOUT_MILLIS 5000

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


// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(DEBUG){
    Serial.print("Last Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");    
  }
}


struct_response response;
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

  
  // Read from Serial
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

  // Send message via ESP-NOW
  success = false;
  //timer for timeout
  unsigned long start = millis();
  esp_err_t result2 = esp_now_send(broadcastAddressAll, (uint8_t *) &myData, sizeof(myData));
  if (result2 == ESP_OK) {  
    //Wait for confirmation from reciever
    while(!success){
      if (millis() - start > TIMEOUT_MILLIS){
        Serial.println("failed");// Sending message failed timeout
        if (DEBUG){
          Serial.println("Timeout");
        }
        break;
      }
    }
    if (success && response.bottleID == bottleID){
      Serial.println("success");// Message received by reciever
    }
    else {
      Serial.println("failed");// Sending message failed revieved wrong bottleID
      if (DEBUG){
        Serial.println("Wrong BottleID");
      }
    }
  }
  else {
    Serial.println("failed");// Sending message failed
  }
  delay(2000);
}

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
