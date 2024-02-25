// Main code to communicate with Cube Pilot and parachute ESPs

#include <Arduino.h>
#include <HardwareSerial.h>
#include <common/mavlink.h>

// #include <Streaming.h>
// #include <SPI.h>
// #include <SD.h>

// #define VERSION "0.0.1"

#include "params.h"
#include "util.h"
#include "linker.h"
#include "autopilot_interface.h"

HardwareSerial PiSerial(2); // UART2
HardwareSerial CubeSerial(2); // UART2

Linker linker;
Autopilot_Interface pixhawk(&linker);

void setup() {
    Serial.begin(57600);
    PiSerial.begin(15200, SERIAL_8N1, 4, 2); 
    CubeSerial.begin(57600, SERIAL_8N1, 16, 17);
    delay(1000);
}

Mavlink_Messages msg;

String drop_state_str;
Drop_State drop_state;
int bottle_num;
double wind[3]; // Wind speed (m/s) in x, y, z (NED)
Drop_State des_drop_state;

void loop() {

    // TODO: put some timeouts on this thing!!

    // Read drop point and bottle number from Pi
    drop_state_str = "";
    while (drop_state_str == "") {
        if (PiSerial.available() > 0) {
            drop_state_str = Serial.readStringUntil('\n');
        }
    }
    drop_state.lat = drop_state_str.substring(0, 5).toDouble();
    drop_state.lon = drop_state_str.substring(5, 10).toDouble();
    drop_state.heading = drop_state_str.substring(10, 15).toDouble();

    bottle_num = drop_state_str.substring(15, 16).toInt();

    // Read wind speed
    msg = pixhawk.read_messages();
    Serial.println(msg.heartbeat.system_status);
    wind[0] = msg.wind.wind_x;
    wind[1] = msg.wind.wind_y;
    wind[2] = msg.wind.wind_z;

    des_drop_state = calc_des_drop_state(wind, drop_state, &des_drop_state);

    // Send message to Pi
    PiSerial.println(String(des_drop_state.lat) + "," + String(des_drop_state.lon) + "," + String(des_drop_state.heading));

    // TODO: Monitor until close enough to drop point and relese payload

}


