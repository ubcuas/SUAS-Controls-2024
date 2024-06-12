#include "encoder.h"

// Definitions of global variables
volatile bool forward_1 = true;
volatile bool forward_2 = true;
volatile unsigned long time_1 = millis();
volatile unsigned long time_2 = millis();
volatile int count_1 = 0;
volatile int count_2 = 0;

// Interrupt service routines
void IRAM_ATTR enc_1_isr() {
  unsigned long current = millis();
  if (current - time_1 > DEBOUNCE_INTERVAL) {
    if (forward_1) { count_1++; }
    else { count_1--; }
    time_1 = current;
  }
}
void IRAM_ATTR enc_2_isr() {
  unsigned long current = millis();
  if (current - time_2 > DEBOUNCE_INTERVAL) {
    if (forward_2) { count_2++; }
    else { count_2--; }
    time_2 = current;
  }
}
