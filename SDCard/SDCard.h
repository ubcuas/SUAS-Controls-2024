#ifndef SDCARD_H
#define SDCARD_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_CS 5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18
#define SD_DETECT 33



namespace SDCard
{

    typdef enum SDCardStatus {
    SDCARD_OK,
    SDCARD_ERROR
    } SDCardStatus;

    SDCardStatus SDcardInit();
    SDCardStatus SDcardWrite(String data); 
} // namespace SDCard



#endif
