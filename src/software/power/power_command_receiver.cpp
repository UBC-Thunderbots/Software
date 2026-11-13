#include "power_command_receiver.h"

#ifdef PLATFORMIO_BUILD
#include "uart_framing_platformio.h"
#else
#include "shared/uart_framing/uart_framing.hpp"
#endif

bool PowerCommandReceiver::receiveByte(uint8_t byte, TbotsProto_PowerFrame& frame)
{
    if (!receiving_)
    {
        if (byte == START_END_FLAG_BYTE)
        {
            buffer_.clear();
            buffer_.push_back(byte);
            receiving_ = true;
        }
        return false;
    }

    if (buffer_.size() >= MAX_PACKET_SIZE)
    {
        buffer_.clear();
        receiving_ = false;
        return false;
    }

    buffer_.push_back(byte);
    if (byte != START_END_FLAG_BYTE)
    {
        return false;
    }

    receiving_             = false;
    const bool valid_frame = unmarshalUartPacket(buffer_, frame);
    buffer_.clear();
    return valid_frame;
}
