#include <ESP32Servo.h>

int servoPin = 27;
 
Servo servo;  
 
void setup() 
{ 
  servo.attach(servoPin); 
} 
 
 
void loop() 
{           
    servo.write(90);           
    delay(15);       
} 