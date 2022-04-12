#include "software/jetson_nano/services/power.h"

#include <cstdint>

#include "shared/uart_framing/uart_framing.hpp"

PowerService::PowerService()
    : status(), READ_BUFFER_SIZE(getMarshalledSize(PowerStatus()))
{
    this->uart = std::make_unique<BoostUartCommunication>(BAUD_RATE, DEVICE_SERIAL_PORT);
}

void PowerService::start() {}

void PowerService::stop() {}

std::unique_ptr<PowerStatus> PowerService::poll(const PowerCommand& command)
{
    auto frame                = createUartMessageFrame(command);
    auto power_command_buffer = frame.marshallUartPacket();

    std::vector<uint8_t> power_status_msg;
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
        power_status_msg = uart->serialRead(READ_BUFFER_SIZE);
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "ESP32 has disconnected. Power service has crashed" << e.what();
    }

    auto uart_frame = UartMessageFrame<PowerStatus>();
    if (unmarshalUartPacket<PowerStatus>(power_status_msg, uart_frame))
    {
        LOG(DEBUG) << "Command status read successfully";
        status = uart_frame.message;
    }
    else
    {
        LOG(WARNING) << "Unmarshal failed";
    }

    return std::make_unique<PowerStatus>(status);
}
