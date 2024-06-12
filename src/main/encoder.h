#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENC_1 25
#define ENC_2 33

#define DEBOUNCE_INTERVAL 10 // ms

// Function prototypes
void enc_1_isr();
void enc_2_isr();

// Declaration of global variables (extern keyword indicates that they are defined elsewhere)
extern volatile bool forward_1;
extern volatile bool forward_2;
extern volatile unsigned long time_1;
extern volatile unsigned long time_2;
extern volatile int count_1;
extern volatile int count_2;

#endif
