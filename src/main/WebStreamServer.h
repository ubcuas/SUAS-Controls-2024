/*
    WebStreamServer.h - Library for streaming data from sensors to the client.
    Name: Nischay Joshi
    Date: 14-12-23
    Description: Header file for WebStreamServer.cpp
    Contains function prototypes for WebStreamServer.cpp 
*/

#ifndef WEBSTREAMSERVER_H
#define WEBSTREAMSERVER_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define SERIAL_PORT Serial


const char html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>ESP Data Logger</title>
    <script>
        let isRecording = false;
        let recordedData = '';
        let headerString = 'Your,Header,String,Here\n'; // Customize this header string

            // Update the WebSocket URL to the IP of your ESP device
            var gateway = `ws://${window.location.hostname}/ws`;
            var websocket;

            // Init web socket when the page loads
            window.addEventListener('load', onload);

            function onload(event) {
            initWebSocket();
            }

            function initWebSocket() {
            console.log('Trying to open a WebSocket connection…');
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
            }

            // When websocket is established, call the getReadings() function
            function onOpen(event) {
              console.log('Connection opened');
            }

            function onClose(event) {
              console.log('Connection closed');
              setTimeout(initWebSocket, 2000);
            }
            
            function onMessage(event) {
                const data = event.data;
                document.getElementById('dataDisplay').innerText = data;
                if (isRecording) {
                    recordedData += data;
                }
            }

        function startRecording() {
            isRecording = true;
            recordedData = headerString; // Start with the header string
            document.getElementById('status').innerText = 'Recording...';
        }

        function stopRecording() {
            isRecording = false;
            document.getElementById('status').innerText = 'Stopped';
            downloadData();
        }

        function downloadData() {
            const blob = new Blob([recordedData], { type: 'text/plain' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'recordedData.txt';
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
        }
    </script>
</head>
<body>
    <h2>ESP Data Logger</h2>
    <div id="dataDisplay">Waiting for data...</div>
    <p id="status">Not Recording</p>
    <button onclick="startRecording()">Start Recording</button>
    <button onclick="stopRecording()">Stop Recording</button>
</body>
</html>
)rawliteral";

class WebStreamServer {
public:
    WebStreamServer(): server(80), ws("/ws"){};
    typedef enum{
        SUCCESS,
        FAILURE,
        NOT_CONNECTED  // Added for completeness
    } WebStreamServerState;

    WebStreamServerState init();
    WebStreamServerState send(char* data);  // Function declaration

private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    bool connected = false;
    const char* ssid = "ESP32-AP";
    const char* password = "12345678";
};

#endif // WEBSTREAMSERVER_H