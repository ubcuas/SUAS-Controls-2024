#include "encoder.h"

void setup() {
  Serial.begin(921600);
  delay(1000);

  pinMode(ENC_1, INPUT);
  attachInterrupt(ENC_1, enc_1_isr, RISING);
  pinMode(ENC_2, INPUT);
  attachInterrupt(ENC_2, enc_2_isr, RISING);
}

void loop() {
  Serial.println((int)count_1 / 2);
  delay(500);

}
