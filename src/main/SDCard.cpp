#include <Arduino.h>
#include "SDCard.h"

SPIClass spi_1 = SPIClass(HSPI);      //define serial_port for serial

namespace SDCard
{
    /**
     * @brief Initialize the SD card
     * We are using the SD card in SPI mode, so we need to connect the SD card to the SPI pins of the ESP32
     * @return SDCardStatus 
    */
    SDCardStatus SDcardInit()
    {
        spi_1.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

        if (!SD.begin(SD_CS, spi_1, 16000000))
        {
            Serial.println("initialization failed!");
            return SDCARD_ERROR;
        }
        Serial.println("initialization done.");
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
        File file = SD.open("/data.txt", FILE_APPEND);
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