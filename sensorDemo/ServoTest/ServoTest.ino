#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
int servoPin = 27; // Default servo pin

int minUS = 500;
int maxUS = 2500;
int angle = 0;

const char HelpMessage[] = R"rawliteral(
  * Control a servo via Serial Terminal Commands
  * - - - - - - - - - - - - - - - - - - - - - - -  
  * Supported Commands:
  * - angle <value> 
  * -- moves the servo to the angle specified. Must be within 0-180 degrees (whole number)
  * - min <value>
  * -- sets the minimum pulse width for the servo
  * - max <value>
  * -- sets the maximum pulse width for the servo  
  * - print
  * -- prints the current set parameter for the servo
  * - help
  * -- prints this message
  *
)rawliteral";

void setup() {
    Serial.begin(921600);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myservo.setPeriodHertz(50);    // standard 50 hz servo
    myservo.attach(servoPin, minUS, maxUS); // Default min/max pulse width
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.print("Command: ");
        Serial.println(input);
        if (input.startsWith("angle")) {
            int newAngle = input.substring(6).toInt();
            if (newAngle >= 0 && newAngle <= 180) {
                angle = newAngle;
                myservo.write(angle);
                Serial.printf("Servo Angle set to: %d\n", angle);
            } else {
                Serial.println("Error: Angle must be within 0-180 degrees.");
            }
        } else if (input.startsWith("min")) {
            int newMinUS = input.substring(4).toInt();
            if (newMinUS <= maxUS) {
                minUS = newMinUS;
                myservo.detach();
                myservo.attach(servoPin, minUS, maxUS);
                Serial.printf("Minimum Pulse Width set to: %d microseconds\n", minUS);
            } else {
                Serial.println("Error: Minimum pulse width cannot exceed maximum pulse width.");
            }
        } else if (input.startsWith("max")) {
            int newMaxUS = input.substring(4).toInt();
            if (newMaxUS >= minUS) {
                maxUS = newMaxUS;
                myservo.detach();
                myservo.attach(servoPin, minUS, maxUS);
                Serial.printf("Maximum Pulse Width set to: %d microseconds\n", maxUS);
            } else {
                Serial.println("Error: Maximum pulse width cannot be less than minimum pulse width.");
            }
        } else if (input.startsWith("print")) {
            Serial.printf("Servo Values now:\nAngle: %d\tMin: %d\tMax: %d\n", angle, minUS, maxUS);
        } else if (input.startsWith("help")){
            Serial.println(HelpMessage);
        } else {
            Serial.print("Unknown Command: ");
            Serial.println(input);
            Serial.println(HelpMessage);
        }
    }
}