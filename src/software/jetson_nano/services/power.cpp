#include "software/jetson_nano/services/power.h"

#include <cstdint>
#include "proto/power_frame_msg.nanopb.h"

#include "shared/uart_framing/uart_framing.hpp"

PowerService::PowerService()
    : READ_BUFFER_SIZE(getMarshalledSize(TbotsProto_PowerStatus TbotsProto_PowerStatus_init_default))
{
    this->uart = std::make_unique<BoostUartCommunication>(io_service, BAUD_RATE,
                                                          DEVICE_SERIAL_PORT);
}

void PowerService::start() {}

void PowerService::stop() {}

std::unique_ptr<TbotsProto::PowerStatus> PowerService::poll(const TbotsProto::PowerControl& command)
{
    auto nanopb_command       = createNanoPbPowerControl(command);
    auto frame                = createUartFrame(nanopb_command);
    auto power_command_buffer = marshallUartPacket(frame);

    std::vector<uint8_t> power_status;
    try
    {
        // Write power command
        if (uart->serialWrite(power_command_buffer))
        {
            uart->flushSerialPort(uart->flush_send);
        }
        else
        {
            LOG(WARNING) << "Writing power command failed.";
        }

        // Read power status
        uart->flushSerialPort(uart->flush_receive);
        power_status = uart->serialRead(READ_BUFFER_SIZE);
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "ESP32 has disconnected. Power service has crashed" << e.what();
    }

    TbotsProto_PowerFrame status_frame = TbotsProto_PowerFrame_init_default;
    if (unmarshalUartPacket(power_status, status_frame))
    {
        LOG(DEBUG) << "Command status read successfully";
        status = status_frame.power_msg.power_status;
    }
    else
    {
        LOG(WARNING) << "Unmarshal failed";
    }

    return createTbotsPowerStatus(status);
}
