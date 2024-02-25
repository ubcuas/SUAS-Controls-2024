// Adapted from https://github.com/Kariboo-Corp/TIMU/tree/main

#ifndef LINKER_H
#define LINKER_H

#include <Arduino.h>
#include <common/mavlink.h>
#include "params.h"
// #include <Streaming.h>
// #include <SPI.h>
// #include <SD.h>

class Linker
{
    public:
        Linker();
        virtual ~Linker();

        int read_message(mavlink_message_t &message);
        int write_message(const mavlink_message_t &message);

        bool is_running() {
            return is_open;
        }

        void stop();
    
    private:
        bool debug = false;
        
        mavlink_status_t lastStatus;
        bool is_open;
        int baudrate = 57600;
};

#endif