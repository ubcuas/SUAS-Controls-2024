#include "GPS.h"

void setup() {
  Serial.begin(115200);
  init_gps();
}

int loopCounter = 0;
void loop() {
  if (updateGPS()) {
    String s = getInfo();
    Serial.println(s);
  }
  loopCounter += 1;
}
