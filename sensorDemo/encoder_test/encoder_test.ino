#define ENC_1 25
#define ENC_2 33

// Global variables for encoders
bool forward_1 = true;
bool forward_2 = true;
volatile unsigned long time_1 = millis();
volatile unsigned long time_2 = millis();
volatile int count_1 = 0;
volatile int count_2 = 0;

void IRAM_ATTR enc_1_isr() {
  unsigned long current = millis();
  if (current - time_1 > 20) { // "Debounce" time interval of 20ms
    if (forward_1) { count_1++; }
    else { count_1--; }
    time_1 = current;
  }
}

void IRAM_ATTR enc_2_isr() {
  unsigned long current = millis();
  if (current - time_2 > 20) { // "Debounce" time interval of 20ms
    if (forward_2) { count_2++; }
    else { count_2--; }
    time_2 = current;
  }
}


void setup() {
  Serial.begin(921600);
  delay(1000);

  pinMode(ENC_1, INPUT);
  attachInterrupt(ENC_1, enc_1_isr, RISING);
  pinMode(ENC_2, INPUT);
  attachInterrupt(ENC_2, enc_2_isr, RISING);
}

void loop() {
  Serial.println(count_1);
  delay(500);

}
