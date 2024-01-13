#include <Arduino.h>
#include <SDCard.h>

namespace SDCard
{
    /**
     * @brief Initialize the SD card
     * We are using the SD card in SPI mode, so we need to connect the SD card to the SPI pins of the ESP32
     * @return SDCardStatus 
    */
    SDCardStatus SDcardInit()
    {

        if (!SD.begin(5))
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
    SDCardStatus SDcardWrite(String data)
    {
        File file = SD.open("/data.txt", FILE_APPEND);
        if (!file)
        {
            Serial.println("Failed to open file for appending");
            return SDCARD_ERROR;
        }
        if (file.print(data))
        {
            Serial.println("File written");
        }
        else
        {
            Serial.println("Write failed");
        }
        file.close();
        return SDCARD_OK;
    }
} // namespace SDCard