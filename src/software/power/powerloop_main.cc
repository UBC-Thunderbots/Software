#ifdef PLATFORMIO_BUILD
#include "charger.h"
#include "chicker.h"
#include "constants_platformio.h"
#include "control_executor.h"
#include "geneva.h"
#include "power_frame_msg_platformio.h"
#include "power_monitor.h"
#include "proto/power_frame_msg.nanopb.h"
#include "proto/robot_status_msg.nanopb.h"
#include "uart_framing_platformio.h"
#else
#include "proto/tbots_nanopb_proto_nanopb_gen/proto/power_frame_msg.nanopb.h"
#include "proto/tbots_nanopb_proto_nanopb_gen/proto/robot_status_msg.nanopb.h"
#include "shared/constants.h"
#include "shared/uart_framing/uart_framing.hpp"
#include "software/power/charger.h"
#include "software/power/chicker.h"
#include "software/power/control_executor.h"
#include "software/power/geneva.h"
#include "software/power/power_monitor.h"

#endif

#include <Arduino.h>

// Used for uart communication
uint8_t incomingByte;
size_t read_buffer_size;
std::vector<uint8_t> buffer;
bool start_found;

uint32_t time_since_last_msg_ms;
uint32_t last_packet_received_time;

std::shared_ptr<Charger> charger;
std::shared_ptr<Chicker> chicker;
std::shared_ptr<PowerMonitor> monitor;
std::shared_ptr<Geneva> geneva;
std::shared_ptr<ControlExecutor> executor;

void setup()
{
    Serial.begin(460800, SERIAL_8N1);
    read_buffer_size = getMarshalledSize(
        TbotsProto_PowerPulseControl TbotsProto_PowerPulseControl_init_default);
    bool start_found          = false;
    time_since_last_msg_ms    = 0;
    last_packet_received_time = 0;
    charger                   = std::make_shared<Charger>();
    chicker                   = std::make_shared<Chicker>();
    monitor                   = std::make_shared<PowerMonitor>();
    geneva                    = std::make_shared<Geneva>();
    executor = std::make_shared<ControlExecutor>(charger, chicker, geneva);
    charger->chargeCapacitors();
}

void loop()
{
    // Read in bytes from Serial and put them into a buffer to read
    while (Serial.available() > 0 && buffer.size() < read_buffer_size)
    {
        incomingByte = Serial.read();

        // Check for end of message found
        if (start_found && incomingByte == START_END_FLAG_BYTE)
        {
            // Handle case where end of packet was read first
            if (buffer.size() != read_buffer_size - 1)
            {
                buffer.clear();
                // End of message correctly found adding to buffer and resetting
            }
            else
            {
                start_found = false;
            }
            buffer.emplace_back(incomingByte);
            // continue adding bytes to buffer
        }
        else if (start_found)
        {
            buffer.emplace_back(incomingByte);
            // Start of message found start adding bytes to buffer
        }
        else if (incomingByte == START_END_FLAG_BYTE)
        {
            buffer.emplace_back(incomingByte);
            start_found = true;
        }
    }
    // Once there is enough data attempt to decode
    if (buffer.size() == read_buffer_size)
    {
        TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
        if (unmarshalUartPacket(buffer, frame))
        {
            // On successful decoding execute the given command
            TbotsProto_PowerPulseControl control = frame.power_msg.power_control;
            last_packet_received_time            = millis();
            executor->execute(control);
        }
        buffer.clear();
    }
    // Read sensor values. These are all instantaneous
    time_since_last_msg_ms = static_cast<int>(millis() - last_packet_received_time);
    auto status            = createNanoPbPowerStatus(
        monitor->getBatteryVoltage(), charger->getCapacitorVoltage(),
        monitor->getCurrentDrawAmp(), geneva->getCurrentSlot(), time_since_last_msg_ms,
        chicker->getBreakBeamTripped());
    auto status_frame = createUartFrame(status);
    auto packet       = marshallUartPacket(status_frame);
    for (auto byte : packet)
    {
        while (Serial.availableForWrite() <= 0)
        {
        }
        Serial.write(byte);
    }
    Serial.flush();

    delay(5);
}
