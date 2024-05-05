// // Encoder class

// #ifndef ENCODER_H
// #define ENCODER_H

// class Encoder {
//   public:

//   Encoder();
//   int get_count();
//   bool forward;
//   volatile unsigned long time;
//   volatile int count;

//   friend void IRAM_ATTR enc_1_isr(); // CAREFUL!! If you modify these in another file, you have to update here!
//   friend void IRAM_ATTR enc_2_isr();

//   private:

// }

// #endif