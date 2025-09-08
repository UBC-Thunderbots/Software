#ifdef PLATFORMIO_BUILD
#include "chicker.h"
#include "constants_platformio.h"
#include "control_executor.h"
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
#include "software/power/chicker.h"
#include "software/power/control_executor.h"
#include "software/power/power_monitor.h"
#endif

#include <Arduino.h>

// Used for uart communication
uint8_t incomingByte;
size_t read_buffer_size;
std::vector<uint8_t> buffer;

uint32_t sequence_num;

std::shared_ptr<Chicker> chicker;
std::shared_ptr<PowerMonitor> monitor;
std::shared_ptr<ControlExecutor> executor;

void setup()
{
    Serial.begin(460800, SERIAL_8N1);
    read_buffer_size = getMarshalledSize(
        TbotsProto_PowerPulseControl TbotsProto_PowerPulseControl_init_default);
    sequence_num = 0;
    chicker      = std::make_shared<Chicker>();
    monitor      = std::make_shared<PowerMonitor>();
    executor     = std::make_shared<ControlExecutor>(chicker);
}

void loop()
{
    // Read in bytes from Serial and put them into a buffer to read
    while (Serial.available() > 0 && buffer.size() < read_buffer_size)
    {
        incomingByte = Serial.read();
        buffer.emplace_back(static_cast<uint8_t>(incomingByte));
    }

    // Once there is enough data attempt to decode
    if (buffer.size() == read_buffer_size)
    {
        TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
        if (unmarshalUartPacket(buffer, frame))
        {
            // On successful decoding execute the given command
            TbotsProto_PowerPulseControl control = frame.power_msg.power_control;
            executor->execute(control);
        }
        buffer.clear();
    }

    // Read sensor values. These are all instantaneous
    auto status = createNanoPbPowerStatus(monitor->getBatteryVoltage(),
                                          monitor->getCurrentDrawAmp(), sequence_num++,
                                          chicker->getBreakBeamTripped());

    // Send out status frame with sensor values
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
