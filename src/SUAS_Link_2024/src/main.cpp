// Main code to communicate with Cube Pilot and parachute ESPs

#include <Arduino.h>
#include <HardwareSerial.h>
#include <common/mavlink.h>
#include <ESP32Servo.h>

#include "params.h"
#include "util.h"
#include "linker.h"
#include "autopilot_interface.h"
#include "Sender.h"

HardwareSerial PiSerial(1); // UART2
HardwareSerial CubeSerial(2); // UART2

Servo servo_front_R;
Servo servo_back_R;
Servo servo_front_L;
Servo servo_back_L;

Linker linker;
Autopilot_Interface pixhawk(&linker);
esp_now_peer_info_t peerInfo;

void setup() {
    Serial.begin(57600);
    PiSerial.begin(57600, SERIAL_8N1, 4, 2); 
    CubeSerial.begin(CUBE_BAUD_RATE, SERIAL_8N1, 16, 17);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    servo_front_R.attach(SERVO_FRONT_R);
    servo_back_R.attach(SERVO_BACK_R);
    servo_front_L.attach(SERVO_FRONT_L);
    servo_back_L.attach(SERVO_BACK_L);
    // Closed positions
    servo_front_R.write(75);
    servo_back_R.write(75);
    servo_front_L.write(75);
    servo_back_L.write(75);
    delay(1000);

    // Set up ESP-NOW communication
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    // Register parachute boards
    memcpy(peerInfo.peer_addr, ADDRESS_1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); }
    memcpy(peerInfo.peer_addr, ADDRESS_2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); }
    memcpy(peerInfo.peer_addr, ADDRESS_3, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); }
    memcpy(peerInfo.peer_addr, ADDRESS_4, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); }
    
    digitalWrite(LED_RED, LOW); // Red LED off indicates comminucation established

    // Request message types from Cube
    for (int i=0; i<5; i++) {
        linker.request_msg(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5);
        delay(250);
        // linker.request_msg(MAVLINK_MSG_ID_WIND_COV, 1);
        linker.request_msg(MAVLINK_MSG_ID_HIGH_LATENCY2, 5);
        delay(250);
        linker.request_msg(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5);
        delay(250);
    }
    digitalWrite(LED_BLUE, LOW); // Blue LED off indicates everything ready
    delay(1000);
    Serial.println("\n\nReady!");
}

Mavlink_Messages msg;
struct_message drop_data; // From Pi
struct_message des_drop_data;
char buffer[100]; // For printing things

void loop() {

    // Read drop point and bottle number from Pi (blocking)
    drop_data = recieveData();

    bool notDropped = true;
    snprintf(buffer, sizeof(buffer), "Received data from Pi: %.8f,%.8f,%.2f,%d\n", drop_data.lat, drop_data.lon, drop_data.heading, drop_data.bottleID); Serial.print(buffer);

    // // Read wind speed
    // msg = pixhawk.read_messages();
    // double windspeed = msg.high_latency2.windspeed;
    // double wind_heading = msg.high_latency2.wind_heading;
    // Serial.println("Wind: " + String(windspeed) + ", " + String(wind_heading));
    // calc_des_drop_state(windspeed, wind_heading, drop_data, &des_drop_data);

    calc_des_drop_state(0, 0, drop_data, &des_drop_data);
    
    // Send desired drop point to Pi
    snprintf(buffer, sizeof(buffer), "%.8f,%.8f,%.2f,%d\n", des_drop_data.lat, des_drop_data.lon, des_drop_data.heading, des_drop_data.bottleID);
    PiSerial.print(buffer);
    Serial.print("Sent desired drop point to Pi (not): "); Serial.print(buffer);

    // Wait to get close enough to desired drop point
    digitalWrite(LED_RED, HIGH);
    while (notDropped) {
        // Read GPS coordinates
        msg = pixhawk.read_messages();
        double lat = (double) msg.global_position_int.lat / 10000000.0;
        double lon = (double) msg.global_position_int.lon / 10000000.0;
        // Serial.println("Coords: " + String(lat, 8) + ", " + String(lon, 8));

        if (msg.servo_output_raw.servo13_raw > 1500) { // If servo 13 is high, then drop the left bottle
            des_drop_data.bottleID = 1; 
            notDropped = false;
            Serial.println("Failsafe mode: Right bottle dropped.");
        } 
        if (msg.servo_output_raw.servo14_raw > 1500) { // If servo 14 is high, then drop the right bottle
            des_drop_data.bottleID = 2; 
            notDropped = false;
            Serial.println("Failsafe mode: Left bottle dropped.");
        } 
        double dist_from_drop_point = distance(lat, lon, des_drop_data.lat, des_drop_data.lon);
        Serial.println("Dist: " + String(dist_from_drop_point));
        if (dist_from_drop_point < RELEASE_MARGIN) {
            notDropped = false;
            Serial.println("Reached drop point");
        }
    }
    digitalWrite(LED_RED, LOW);

    if (des_drop_data.bottleID == 1 || des_drop_data.bottleID == 3 || des_drop_data.bottleID == 5) {
        servo_front_R.write(160);
        servo_back_R.write(170);
    }
    else if (des_drop_data.bottleID == 2 || des_drop_data.bottleID == 4) {
        servo_front_L.write(150);
        servo_back_L.write(160);
    }

    PiSerial.println(String(des_drop_data.bottleID)); // Tell software bottle has dropped

    delay(1000);

}
