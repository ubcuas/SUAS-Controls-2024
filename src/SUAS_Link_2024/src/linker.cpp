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
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    uint8_t len = mavlink_msg_to_send_buffer(buf, &message);
    CubeSerial.write(buf, len);

    return len;
}

/*
 * Packs a message and sends to Cube to request a message type from Cube
 * https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
 * https://github.com/mavlink/c_library_v2/blob/103b846f287b9258f175e5c98ff9184e23d3cb39/common/mavlink_msg_command_long.h
 * 
 * @param msg_id
 * @param frequency in Hz
 */
void Linker::request_msg(int msg_id, int frequency) {
    mavlink_message_t request;
    mavlink_msg_command_long_pack(
        0, // system_id (anything but 1 or 255)
        0, // component_id (random)
        &request, 
        1, // target_system
        0, // target_component
        MAV_CMD_SET_MESSAGE_INTERVAL, // command
        0, // confirmation
        msg_id, // ID of message requested
        1000000/frequency, // 10 Hz
        0, 0, 0, 0, 0 // Other stuff we don't need
    );
    write_message(request);
}

void Linker::stop()
{
    CubeSerial.end();
}