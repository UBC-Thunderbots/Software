#include "software/jetson_nano/services/power.h"

#include <boost/bind/bind.hpp>
#include <boost/filesystem.hpp>
#include <cstdint>

#include "proto/power_frame_msg.nanopb.h"

PowerService::PowerService()
{
    if (!boost::filesystem::exists(DEVICE_SERIAL_PORT))
    {
        LOG(FATAL) << "PLUG THE USB INTO THE JETSON NANO";
    }
    this->uart = std::make_unique<BoostUartCommunication>(BAUD_RATE, DEVICE_SERIAL_PORT);
    this->read_thread = std::thread(boost::bind(&PowerService::continuousRead, this));
}

PowerService::~PowerService()
{
    read_thread.join();
}

void PowerService::continuousRead()
{
    for (;;)
    {
        tick();
    }
}

void PowerService::tick()
{
    std::vector<uint8_t> power_status;
    try
    {
        uart->flushSerialPort(uart->flush_receive);
        power_status = uart->serialRead(READ_BUFFER_SIZE);
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "Read thread has crashed" << e.what();
    }

    TbotsProto_PowerFrame status_frame = TbotsProto_PowerFrame_init_default;
    if (!unmarshalUartPacket(power_status, status_frame))
    {
        LOG(WARNING) << "Unmarshal failed";
    }
    else
    {
        status = status_frame.power_msg.power_status;
    }

    auto command =
        nanopb_command.load(std::memory_order_relaxed);  // get value atomically
    auto frame                = createUartFrame(command);
    auto power_command_buffer = marshallUartPacket(frame);

    try
    {
        // Write power command
        uart->flushSerialPort(uart->flush_send);
        if (!uart->serialWrite(power_command_buffer))
        {
            LOG(WARNING) << "Writing power command failed.";
        }
    }
    catch (std::exception& e)
    {
        LOG(FATAL) << "ESP32 has disconnected. Power service has crashed" << e.what();
    }
}

TbotsProto::PowerStatus PowerService::poll(const TbotsProto::PowerControl& command,
                                           double kick_coeff, int kick_constant,
                                           double chip_coeff, int chip_constant)
{
    // Store msg for later transmission
    nanopb_command =
        createNanoPbPowerPulseControl(command, kick_coeff, kick_constant, chip_coeff, chip_constant);
    return *createTbotsPowerStatus(status);
}
