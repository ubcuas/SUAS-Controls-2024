#include <WiFi.h>
#include "GPS.h"

#define RECORD_LENGTH_SECONDS 10

const char* ssid = "Tayyibsphone";
const char* password = "tayyib12345";
const char* host = "192.168.173.137"; // Replace with your computer's IP address
const uint16_t port = 12345;

void setup() {
  Serial.begin(115200);
  init_gps();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  WiFiClient client;
  
  if (client.connect(host, port)) {
    unsigned long currentTime = millis();
    String data = "Starting! at ";
    data += currentTime;
    // client.println(data);
    while(millis() - currentTime < RECORD_LENGTH_SECONDS * 1000){
      updateGPS();
      String val = getInfo();   
      client.printf("%d, %s;\n", millis(), val.c_str()); 
      Serial.printf("%d, %s;\n", millis(), val.c_str());
    }
    client.stop();
  }
  delay(1000); // send every 1 second
}

