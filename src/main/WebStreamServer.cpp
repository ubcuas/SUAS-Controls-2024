/*
    WebStreamServer.cpp - Library for streaming data from ESP32 to a web browser.
    Name: Nischay Joshi
    Date: 14-12-23
    Description: Header file for WebStreamServer.cpp
    Contains function prototypes for WebStreamServer.cpp 
*/

#include "WebStreamServer.h"

WebStreamServer::WebStreamServerState WebStreamServer::init(){
    // Set up the ESP32 as an Access Point
    // WiFi.softAP(this->ssid, this->password);
    IPAddress IP;
    WiFi.begin(ssid, password);
    SERIAL_PORT.print("\n");
    int i = 0;
    bool softAP = false;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        SERIAL_PORT.print(".");
        i++;
        if(i > 5){
          SERIAL_PORT.println("\nCannot connect to Wifi, starting accesspoint. 3d Plotting not available");
          WiFi.disconnect(); // Disconnect from any existing Wi-Fi connections or attempts
          delay(100); // Short delay to allow the WiFi hardware to initialize the change
          softAP = true;
          break;
        }
    }
    if(softAP){
      WiFi.softAP(this->ssid, this->password);
      delay(500);
      IP = WiFi.softAPIP();
    }
    else{
      IP = WiFi.localIP();
    }
    
    SERIAL_PORT.print("\nDevice Connected!\nAP IP address: ");
    SERIAL_PORT.println(IP);
    delay(1000);

    // WebSocket Event Handler with correct lambda capture
    this->ws.onEvent([this](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            //SERIAL_PORT.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            this->connected = true;
        } else if (type == WS_EVT_DISCONNECT) {
            //SERIAL_PORT.printf("WebSocket client #%u disconnected\n", client->id());
            this->connected = false;
        }
        // Handle other events (data, pong, error) if necessary
    });
    this->server.addHandler(&this->ws);

    // HTTP handler
    this->server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", html_page);  // Ensure html_page is defined
    });
    // Define a new route for the custom function
    this->server.on("/customAction", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (customFunction) {
            customFunction(); // Call the custom function
        }
        request->send(200, "text/plain", "Custom action executed");
    });

    // Start server
    this->server.begin();

    return WebStreamServerState::SUCCESS;
}

// Corrected function name to match the declaration
WebStreamServer::WebStreamServerState WebStreamServer::send(char *data){
    // Send data if connected
    if(this->connected == false){
        return WebStreamServerState::NOT_CONNECTED;
    }
    this->ws.textAll(data);
    return WebStreamServerState::SUCCESS;
}
