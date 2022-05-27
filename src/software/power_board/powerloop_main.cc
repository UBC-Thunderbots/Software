#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>  // PlatformIO sees and includes the library based on the bazel rule name ONLY
#include <pb_decode.h>
#include <pb_encode.h>
#include <power_frame_msg_platformio.h>
#include <proto/power_frame_msg.nanopb.h>
#include <proto/primitive.nanopb.h>
#include <proto/robot_status_msg.nanopb.h>
#include <uart_framing_platformio.h>
#else
#include <constants_platformio.h>

#include "proto/tbots_nanopb_proto_nanopb_gen/proto/power_frame_msg.nanopb.h"
#include "proto/tbots_nanopb_proto_nanopb_gen/proto/primitive.nanopb.h"
#include "proto/tbots_nanopb_proto_nanopb_gen/proto/robot_status_msg.nanopb.h"
#include "shared/constants.h"
#include "shared/uart_framing/uart_framing.hpp"
#endif

#include <Arduino.h>

#define RXD2 9
#define TXD2 10

std::vector<uint8_t> buffer;
int incomingByte;
size_t read_buffer_size;
std::vector<uint8_t> packet;

void setup()
{
    Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);
    read_buffer_size =
        getMarshalledSize(TbotsProto_PowerControl TbotsProto_PowerControl_init_default);
}

void loop()
{
    if (Serial.available() > 0)
    {
        incomingByte = Serial.read();
        buffer.emplace_back(static_cast<uint8_t>(incomingByte));
    }
    if (buffer.size() == read_buffer_size)
    {
        TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
        if (unmarshalUartPacket(buffer, frame))
        {
            // TODO(#2597): Perform command here

            auto status = createNanoPbPowerStatus(1.2f, 2.3f, 3.4f, 4.5f, true, false);
            auto status_frame = createUartFrame(status);
            packet            = marshallUartPacket(status_frame);
        }
    }
    if (!packet.empty())
    {
        for (auto byte : packet)
        {
            if (Serial.availableForWrite() > 0)
            {
                Serial.write(byte);
            }
        }
        delay(25);
    }
}
