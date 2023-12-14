#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// Replace with your desired credentials
const char* ssid = "ESP32-AP";
const char* password = "12345678";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
bool connected = false;

volatile int mycounter = 0;

const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>ESP Data Logger</title>
    <script>
        let isRecording = false;
        let recordedData = '';

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
                    recordedData += data + '\n';
                }
            }

        function startRecording() {
            isRecording = true;
            recordedData = ''; // Reset previously recorded data
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



void setup() {
    Serial.begin(115200);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Set up the ESP32 as an Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // WebSocket Event Handler
    ws.onEvent([](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
        if (type == WS_EVT_CONNECT) {
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            connected = true;
        } else if (type == WS_EVT_DISCONNECT) {
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            connected = false;
        }
        // Handle other events (data, pong, error) if necessary
    });
    server.addHandler(&ws);

    // Serve the HTML page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", html);
    });

    // Start server
    server.begin();
}

void loop() {
    static unsigned long lastTime = 0;  // Last time measurements were sent
    unsigned long now = millis();

    // Send data every 250ms
    if ((connected) && (now - lastTime >= 18)) {
        lastTime = now;  // Update the last time

        // Create CSV-formatted data string
        // Replace these values with actual sensor readings if necessary
        String csvData = String((int)mycounter) + "," + String(random(0, 100)) + "," + String(random(0, 100)) + "," + String(random(0, 100));

        // Send the data to all connected WebSocket clients
        ws.textAll(csvData);
        Serial.println(mycounter);
        mycounter++;

    }
}
