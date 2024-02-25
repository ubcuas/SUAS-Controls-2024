// Adapted from https://github.com/Kariboo-Corp/TIMU/tree/main

#include "linker.h"

Linker::Linker()
{
    this->baudrate = baudrate;
    this->is_open = true;
}

Linker::~Linker() 
{
    ;
}

int Linker::read_message(mavlink_message_t &message) 
{
    mavlink_status_t status;
    uint8_t cp;
    uint8_t msgReceived = false;

    if (CubeSerial.available() > 0) 
    {
        cp = CubeSerial.read();

        if (1 > 0) // Specific arduino trick to force message reading
        {
            msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);

            if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
            {
                Serial.println("ERROR: DROPPED " + String(status.packet_rx_drop_count) + "PACKETS");
            }

            lastStatus = status;
        } else 
        {
            Serial.println("ERROR: Coulnd't read from port");
        }

        if(msgReceived && debug)
        {
            // Report info
            Serial.println("Received message from serial with ID #" + String(message.msgid) + " (sys:" + String(message.sysid) + "|comp:" + String(message.compid) + "):");
            Serial.print("Received serial data:");
            unsigned int i;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

            // check message is write length
            unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

            // message length error
            if (messageLength > MAVLINK_MAX_PACKET_LEN)
            {
                Serial.println("\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE");
            }

            // print out the buffer
            else
            {
                for (i=0; i<messageLength; i++)
                {
                    unsigned char v=buffer[i];
                    Serial.print(v);
                }
                Serial.println();
            }
        }

        // Done!
        return msgReceived;
    }
    
    return -1;
}

int Linker::write_message(const mavlink_message_t &message)
{
    char buf[300];

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    CubeSerial.write(buf);

    return len;
}

void Linker::stop()
{
    CubeSerial.end();
}