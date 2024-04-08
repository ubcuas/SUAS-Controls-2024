#include <Arduino.h>
#include "SDCard.h"

SPIClass spi_1 = SPIClass(HSPI);      //define serial_port for serial

SDCard::SDCardStatus SDStatus;

namespace SDCard
{
     
    
    /**
     * @brief Initialize the SD card
     * We are using the SD card in SPI mode, so we need to connect the SD card to the SPI pins of the ESP32
     * @return SDCardStatus 
    */
    SDCardStatus SDcardInit()
    {
        //configure the SD card detect pin as digital input:
        pinMode(SD_DETECT, INPUT);
        delay(10);
        
        if(digitalRead(SD_DETECT)){
          Serial.println("SD Card Not Plugged in!!\n");
          SDStatus = SDCARD_NOTCONNECTED;
          return SDCARD_NOTCONNECTED;
        }

        spi_1.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

        if (!SD.begin(SD_CS, spi_1, 16000000))
        {
            Serial.println("initialization failed!");
            SDStatus = SDCARD_ERROR;
            return SDCARD_ERROR;
        }
        Serial.println("initialization done.");
        // return SDCARD_OK;
        SDStatus = SDCARD_OK;
        return CheckAndRenameFiles();
    }

    /**
     * @brief Check if the file exists and rename it
     * @return SDCardStatus 
     */
    SDCardStatus CheckAndRenameFiles(){
        File root = SD.open(LOG_DIR);
        int maxNum = 0;
        bool basefileExists = false;

        while(true){
            File entry = root.openNextFile();
            if(!entry){
                break;
            }
            if(entry.isDirectory()){
                continue;
            }
            String fileName = entry.name();
            Serial.println(fileName);
            if(fileName == (String(LOG_FILE_BASE) + String(LOG_FILE_EXT))){
                basefileExists = true;
            }
            else if (fileName.startsWith(String(LOG_FILE_BASE)) && fileName.endsWith("_old" + String(LOG_FILE_EXT))) {
                int startOfNum = fileName.lastIndexOf('_', fileName.lastIndexOf("_old") - 1) + 1;
                int endOfNum = fileName.lastIndexOf("_old");
                if (startOfNum >= 0 && endOfNum > startOfNum) {
                    String numStr = fileName.substring(startOfNum, endOfNum);
                    int num = numStr.toInt();
                    if (num > maxNum) {
                        maxNum = num;
                    }
                    Serial.printf("MaxNum: %d\n", maxNum);
                }
            }
            entry.close();
        }
        root.close();

        if(basefileExists){
            String oldFileName = String(LOG_DIR) + "/" + String(LOG_FILE_BASE) + String(LOG_FILE_EXT);
            String newFileName = String(LOG_DIR) + "/" + String(LOG_FILE_BASE) + "_" + String(maxNum + 1) + "_old" + String(LOG_FILE_EXT);
            bool renameResult = SD.rename(oldFileName.c_str(), newFileName.c_str());
            Serial.println(renameResult ? "Rename successful" : "Rename failed");
            if (!renameResult) {
                // Handle rename failure
                SDStatus = SDCARD_ERROR;
                return SDCARD_ERROR;
            }
        }

        // create a new base file
        File file = SD.open((String(LOG_DIR) + "/" + String(LOG_FILE_BASE) + String(LOG_FILE_EXT)), FILE_WRITE);
        if(!file){
            Serial.println("Failed to create new file");
            SDStatus = SDCARD_ERROR;
            return SDCARD_ERROR;
        }
        file.close();

        Serial.println("Check and rename done");
        Serial.printf("Current Old Files: %d\n", maxNum);
        return SDCARD_OK;
    }

    /**
     * @brief Write data to the SD card
     * We assume that the SD card is already initialized
     * @param data the data to write to the SD card 
     * @return SDCardStatus 
     */
    SDCardStatus SDcardWrite(const char data[])
    { 
        if(digitalRead(SD_DETECT)){
          return SDCARD_NOTCONNECTED;
        }
        File file = SD.open(
                (String(LOG_DIR) + "/" + String(LOG_FILE_BASE) + String(LOG_FILE_EXT)), FILE_APPEND
            );
        if (!file)
        {
            // Serial.println("Failed to open file for appending");
            return SDCARD_ERROR;
        }
        if (file.print(data))
        {
            // Serial.println("File written");
        }
        else
        {
            // Serial.println("Write failed");
        }
        file.close();
        return SDCARD_OK;
    }
} // namespace SDCard