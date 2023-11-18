#include <WiFi.h>
#include "GPS.h"
#include "imu.h"
#include "bmp_280.h"

#define RECORD_LENGTH_SECONDS 10

const char* ssid = "ali-phone";
const char* password = "chpx0435";
const char* host = "192.168.245.142"; // Replace with your computer's IP address
const uint16_t port = 12345;

void setup() {
  Serial.begin(115200);
  init_gps();
  init_bno055();
  init_bmp_280();

  // Enter any serial input to continue
  Serial.println("Enter a host IP in the first 10 seconds");
  while (millis() < 10000) {
    if (Serial.available() > 0) {
      String result = Serial.readString();
      if (result.startsWith("IP: ")) {
        String hostStr = result.substring(4);
        host = hostStr.c_str();
        Serial.printf("Host IP changed to %s!", host);
      } else {
        Serial.println("Error: host IP should begin with 'IP: ', then the host's public IPV4 address.");
        Serial.printf("Reverting to default host: %s\n", host);
      }
      break;
    }
  }
  Serial.printf("No IP entered. Reverting to default host: %s\n", host);

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

      String val = getInfo() + ",";
      String imu_data = imu_getInfo() + ",";
      String bmp_280_data = bmp_280_getInfo();
      val += imu_data;    
      val += bmp_280_data;

      client.printf("%d, %s;\n", millis(), val.c_str()); 
      Serial.printf("%d, %s;\n", millis(), val.c_str());
    }
    client.stop();
  }
  delay(1000); // send every 1 second
}

