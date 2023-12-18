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
    // IPAddress IP = WiFi.softAPIP();
    WiFi.begin(ssid, password);
    Serial.print("\n");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    IPAddress IP = WiFi.localIP();
    SERIAL_PORT.print("\nDEvice Connected!\nAP IP address: ");
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
