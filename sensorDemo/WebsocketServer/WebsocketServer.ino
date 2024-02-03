#include "WebStreamServer.h"

volatile int mycounter = 0;
WebStreamServer webStreamServer_inst;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Web Stream Server");
    webStreamServer_inst.init();
    Serial.println("Web Stream Server Started");
}

void loop() {
    static unsigned long lastTime = 0;  // Last time measurements were sent
    unsigned long now = millis();

    // Send data every 250ms
    if ((now - lastTime >= 18)) {
        lastTime = now;  // Update the last time

         // Create a character buffer for the CSV data
        char csvData[50]; // Adjust the size as necessary

        // Format the data into the buffer
        snprintf(csvData, sizeof(csvData), "%d,%d,%d,%d", mycounter, random(0, 100), random(0, 100), random(0, 100));
        // Send the data to all connected WebSocket clients
        if(webStreamServer_inst.send(csvData) == WebStreamServer::SUCCESS){
          Serial.println(mycounter);
          mycounter++;
        }
    }
}
