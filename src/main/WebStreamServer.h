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
#include <functional>


#define SERIAL_PORT Serial


const char html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>ESP Data Logger with 3D Plot</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <script>
        let isRecording = false;
        let recordedData = '';
        let headerString = 'Your,Header,String,Here\n'; // Customize this header string
        let plotData = [];

        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
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
            plotData.push(data.split(','));
            updatePlot();
            if (isRecording) {
                recordedData += data + '\n';
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

        function callCustomFunction() {
            fetch('/customAction')
            .then(response => response.text())
            .then(data => console.log(data));
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

        function updatePlot() {
    let xAxis = document.getElementById('xAxis').value;
    let yAxis = document.getElementById('yAxis').value;
    let zAxis = document.getElementById('zAxis').value;

    let trace = {
        x: plotData.map(d => parseFloat(d[xAxis])),
        y: plotData.map(d => parseFloat(d[yAxis])),
        z: plotData.map(d => parseFloat(d[zAxis])),
        mode: 'lines', // Changed from 'markers' to 'lines'
        type: 'scatter3d'
    };

    let layout = {
        margin: {
            l: 0,
            r: 0,
            b: 0,
            t: 0
        }
    };

    Plotly.newPlot('plot', [trace], layout);
}

function clearPlot() {
    // Clear the plotData array
    plotData = [];

    // Optionally, clear the plot display as well
    Plotly.newPlot('plot', [], {});
}
    </script>
</head>
<body>
    <h2>ESP Data Logger</h2>
    <div id="dataDisplay">Waiting for data...</div>
    <p id="status">Not Recording</p>
    <button onclick="startRecording()">Start Recording</button>
    <button onclick="stopRecording()">Stop Recording</button>
    <!-- 'Clear Plot' Button -->
    <button onclick="clearPlot()">Clear Plot</button>
    <!-- 'Reset Reference Button -->
    <button onclick="callCustomFunction()">Reset GPS Reference</button>


<!-- Dropdown Menus for Axis Selection -->
<p>Select data channels for each axis:</p>
<select id="xAxis" onchange="updatePlot()">
    <option value="0" selected>Channel 1</option>
    <option value="1">Channel 2</option>
    <option value="2">Channel 3</option>
    <!-- Add more options based on your data channels -->
</select>
<select id="yAxis" onchange="updatePlot()">
    <option value="0">Channel 1</option>
    <option value="1" selected>Channel 2</option>
    <option value="2">Channel 3</option>
    <!-- Add more options -->
</select>
<select id="zAxis" onchange="updatePlot()">
    <option value="0">Channel 1</option>
    <option value="1">Channel 2</option>
    <option value="2" selected>Channel 3</option>
    <!-- Add more options -->
</select>

    <!-- Div for Plotting -->
    <div id="plot"></div>
</body>
</html>
)rawliteral";

class WebStreamServer {
public:
    typedef std::function<void()> CustomFunction;
    WebStreamServer(): server(80), ws("/ws"){};
    typedef enum{
        SUCCESS,
        FAILURE,
        NOT_CONNECTED  // Added for completeness
    } WebStreamServerState;

    WebStreamServerState init();
    WebStreamServerState send(char* data);  // Function declaration
    void setCustomFunction(CustomFunction func) {
        this->customFunction = func;
    }

private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    bool connected = false;
    const char* ssid = "Nischay_iphone";
    const char* password = "r18nmbr4";
    CustomFunction customFunction;
};

#endif // WEBSTREAMSERVER_H