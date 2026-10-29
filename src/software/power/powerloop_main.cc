#ifdef PLATFORMIO_BUILD
#include "constants_platformio.h"
#include "control_executor.h"
#include "esp_hal.h"
#include "power_controller.h"
#include "power_frame_msg_platformio.h"
#include "proto/power_frame_msg.nanopb.h"
#include "proto/robot_status_msg.nanopb.h"
#include "uart_framing_platformio.h"
#else
#include "proto/power_frame_msg.nanopb.h"
#include "proto/robot_status_msg.nanopb.h"
#include "shared/constants.h"
#include "shared/uart_framing/uart_framing.hpp"
#include "software/power/control_executor.h"
#include "software/power/esp/esp_hal.h"
#include "software/power/power_controller.h"
#endif

#include <Arduino.h>

#include <memory>
#include <vector>

#ifdef DEBUG_POWERLOOP
/**
 * This section of code is reserved for DIRECT testing on the microcontroller/debug
 * functionality.
 */
void setup() {}
void loop() {}
#else

// Used for uart communication
std::vector<uint8_t> buffer;
bool receiving;

uint32_t sequence_num;

std::shared_ptr<EspHal> power_hal;
std::shared_ptr<PowerController> power_controller;
std::shared_ptr<ControlExecutor> executor;

void setup()
{
    Serial.begin(460800, SERIAL_8N1);
    receiving        = false;
    sequence_num     = 0;
    power_hal        = std::make_shared<EspHal>();
    power_controller = std::make_shared<PowerController>(*power_hal);
    executor         = std::make_shared<ControlExecutor>(*power_controller);
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
            if (byte == START_END_FLAG_BYTE)  // End of packet?
            {
                TbotsProto_PowerFrame frame = TbotsProto_PowerFrame_init_default;
                if (unmarshalUartPacket(buffer, frame))
                {
                    // On successful decoding execute the given command
                    if (frame.which_power_msg == TbotsProto_PowerFrame_power_control_tag)
                    {
                        executor->execute(frame.power_msg.power_control);
                    }
                    else if (frame.which_power_msg ==
                             TbotsProto_PowerFrame_dribbler_control_tag)
                    {
                        power_controller->setDribblerTargetSpeed(
                            frame.power_msg.dribbler_control.dribbler_speed);
                    }
                }

                buffer.clear();
                receiving = false;
            }
        }
        else
        {
            if (byte == START_END_FLAG_BYTE)  // Start of packet?
            {
                buffer.push_back(byte);
                receiving = true;
            }
        }
    }

    power_controller->update();

    // Read sensor values. These are all instantaneous
    TbotsProto_PowerStatus status = createNanoPbPowerStatus(
        power_hal->getBatteryVoltage(), power_hal->getCapacitorVoltage(),
        power_hal->getCurrentDrawAmp(), sequence_num++,
        power_hal->getBreakBeamTripped());

    // Write sensor values out to Serial
    TbotsProto_PowerFrame status_frame = createUartFrame(status);
    std::vector<uint8_t> packet        = marshallUartPacket(status_frame);
    Serial.write(packet.data(), packet.size());
    Serial.flush();

    delay(5);
}
#endif
