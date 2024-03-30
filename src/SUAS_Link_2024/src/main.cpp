// Main code to communicate with Cube Pilot and parachute ESPs

#include <Arduino.h>
#include <HardwareSerial.h>
#include <common/mavlink.h>
#include <ESP32Servo.h>

// #include <Streaming.h>
// #include <SPI.h>
// #include <SD.h>

// #define VERSION "0.0.1"

#include "params.h"
#include "util.h"
#include "linker.h"
#include "autopilot_interface.h"

HardwareSerial PiSerial(1); // UART2
HardwareSerial CubeSerial(2); // UART2

Servo servo_front_R;
Servo servo_back_R;
Servo servo_front_L;
Servo servo_back_L;

Linker linker;
Autopilot_Interface pixhawk(&linker);

void setup() {
    Serial.begin(57600);
    PiSerial.begin(15200, SERIAL_8N1, 4, 2); 
    CubeSerial.begin(57600, SERIAL_8N1, 16, 17);
    delay(1000);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    servo_front_R.attach(SERVO_FRONT_R);
    servo_back_R.attach(SERVO_BACK_R);
    servo_front_L.attach(SERVO_FRONT_L);
    servo_back_L.attach(SERVO_BACK_L);

    // Request message types
    for (int i=0; i<5; i++) {
        linker.request_msg(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1);
        delay(200);
        // linker.request_msg(MAVLINK_MSG_ID_WIND_COV, 1);
        linker.request_msg(MAVLINK_MSG_ID_HIGH_LATENCY2, 1);
        delay(200);
    }
    

    Serial.println("\n\nReady! :D");
}

Mavlink_Messages msg;

String drop_state_str;
Drop_State drop_state;
int bottle_num;
double wind[3]; // Wind speed (m/s) in x, y, z (NED)
Drop_State des_drop_state;

void loop() {

    // // TODO: put some timeouts on this thing!!

    // // Read drop point and bottle number from Pi
    // drop_state_str = "";
    // while (drop_state_str == "") {
    //     if (PiSerial.available() > 0) {
    //         drop_state_str = PiSerial.readStringUntil('\n');
    //     }
    // }
    // drop_state.lat = drop_state_str.substring(0, 5).toDouble();
    // drop_state.lon = drop_state_str.substring(5, 10).toDouble();
    // drop_state.heading = drop_state_str.substring(10, 15).toDouble();

    // bottle_num = drop_state_str.substring(15, 16).toInt();

    // Read wind speed
    msg = pixhawk.read_messages();
    // Serial.println(msg.heartbeat.system_status);
    Serial.println(msg.global_position_int.lat);
    // Serial.println(msg.wind.wind_x);
    Serial.println(msg.high_latency2.windspeed);
    // Serial.println(msg.attitude.pitch);
    // Serial.println(msg.sys_status.onboard_control_sensors_enabled);

    // wind[0] = msg.wind.wind_x;
    // wind[1] = msg.wind.wind_y;
    // wind[2] = msg.wind.wind_z;

    // des_drop_state = calc_des_drop_state(wind, drop_state, &des_drop_state);

    // // Send message to Pi
    // PiSerial.println(String(des_drop_state.lat) + "," + String(des_drop_state.lon) + "," + String(des_drop_state.heading));

    // // TODO: Monitor until close enough to drop point and relese payload

    // servo_front_R.write(0);
    // servo_back_R.write(0);
    // servo_front_L.write(0);
    // servo_back_L.write(0);0]'
    // delay(1000);

    // servo_front_R.write(180);
    // servo_back_R.write(180);
    // servo_front_L.write(180);
    // servo_back_L.write(180);
    // delay(1000);



}


