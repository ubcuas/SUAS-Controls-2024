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
#define SD_DETECT 17

#define LOG_FILE_BASE "data"
#define LOG_FILE_EXT ".txt"
#define LOG_DIR "/log"


namespace SDCard
{

    typedef enum SDCardStatus {   //changed here
    SDCARD_OK,
    SDCARD_ERROR,
    SDCARD_NOTCONNECTED
    } SDCardStatus;

    SDCardStatus SDcardInit();
    SDCardStatus CheckAndRenameFiles();
    SDCardStatus SDcardWrite(const char data[]); 
} // namespace SDCard



#endif