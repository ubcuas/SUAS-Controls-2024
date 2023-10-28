#include <WiFi.h>

const char* ssid = "Tayyibsphone";
const char* password = "tayyib12345";
const char* host = "192.168.173.137"; // Replace with your computer's IP address
const uint16_t port = 12345;

void setup() {
  Serial.begin(115200);

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
    // Replace with your data reading logic
    String data = "Hello from ESP32!";

    client.println(data);
    for(int i = 0; i < 10; i++){
      client.println(getVal());
    } 
    client.println();
    client.println("end");
    client.stop();
  }
  delay(1000); // send every 1 second
}

int getVal(){
  return random(0, 100);
}
