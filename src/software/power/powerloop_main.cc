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
#include "proto/power_frame_msg.nanopb.h"
#include "proto/robot_status_msg.nanopb.h"
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
std::vector<uint8_t> buffer;
bool receiving;

uint32_t sequence_num;

std::shared_ptr<Charger> charger;
std::shared_ptr<Chicker> chicker;
std::shared_ptr<PowerMonitor> monitor;
std::shared_ptr<Geneva> geneva;
std::shared_ptr<ControlExecutor> executor;

void setup()
{
    Serial.begin(460800, SERIAL_8N1);
    receiving    = false;
    sequence_num = 0;
    charger      = std::make_shared<Charger>();
    chicker      = std::make_shared<Chicker>();
    monitor      = std::make_shared<PowerMonitor>();
    geneva       = std::make_shared<Geneva>();
    executor     = std::make_shared<ControlExecutor>(charger, chicker, geneva);
    charger->chargeCapacitors();
}

void loop()
{
    // Read bytes in from Serial
    while (Serial.available() > 0)
    {
        const uint8_t byte = Serial.read();
        if (receiving)
        {
            buffer.push_back(byte);
            if (byte == START_END_FLAG_BYTE) // End of packet?
            {
                TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
                if (unmarshalUartPacket(buffer, frame))
                {
                    // On successful decoding execute the given command
                    TbotsProto_PowerPulseControl control = frame.power_msg.power_control;
                    executor->execute(control);
                }

                buffer.clear();
                receiving = false;
            }
        }
        else
        {
            if (byte == START_END_FLAG_BYTE) // Start of packet?
            {
                buffer.push_back(byte);
                receiving = true;
            }
        }
    }

    // Read sensor values. These are all instantaneous
    TbotsProto_PowerStatus status = createNanoPbPowerStatus(
        monitor->getBatteryVoltage(), charger->getCapacitorVoltage(),
        monitor->getCurrentDrawAmp(), geneva->getCurrentSlot(), sequence_num++,
        chicker->getBreakBeamTripped());

    // Write sensor values out to Serial
    TbotsProto_PowerFrame status_frame = createUartFrame(status);
    std::vector<uint8_t> packet        = marshallUartPacket(status_frame);
    Serial.write(packet.data(), packet.size());
    Serial.flush();

    delay(5);
}
