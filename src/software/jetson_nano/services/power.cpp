#include "software/jetson_nano/services/power.h"

#include <boost/bind/bind.hpp>
#include <cstdint>

#include "shared/uart_framing/uart_framing.hpp"

PowerService::PowerService()
    : timer(io_service, boost::posix_time::milliseconds(STARTUP_TIME_MS)),
      status(),
      READ_BUFFER_SIZE(getMarshalledSize(PowerStatus()))
{
    this->uart        = std::make_unique<BoostUartCommunication>(io_service, BAUD_RATE,
                                                          DEVICE_SERIAL_PORT);
    this->read_thread = std::thread(boost::bind(&PowerService::continuousRead, this));
}

PowerService::~PowerService()
{
    in_destructor = true;
    read_thread.join();
}

void PowerService::start() {}

void PowerService::stop() {}

void PowerService::continuousRead()
{
    timer.async_wait(
        boost::bind(&PowerService::tick, this, boost::asio::placeholders::error));
    io_service.run();
}

void PowerService::tick(const boost::system::error_code& error)
{
    if (in_destructor)
    {
        timer.cancel();
    }
    else
    {
        std::vector<uint8_t> power_status_msg;
        try
        {
            uart->flushSerialPort(uart->flush_receive);
            power_status_msg = uart->serialRead(READ_BUFFER_SIZE);
        }
        catch (std::exception& e)
        {
            LOG(FATAL) << "Read thread has crashed" << e.what();
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

        boost::posix_time::milliseconds next_interval(INTERVAL_BETWEEN_READS_MS);

        // Reschedule the timer for interval seconds in the future:
        timer.expires_from_now(next_interval);
        // Posts the timer event
        timer.async_wait(
            boost::bind(&PowerService::tick, this, boost::asio::placeholders::error));
    }
}

std::unique_ptr<PowerStatus> PowerService::poll(const PowerCommand& command)
{
    auto frame        = createUartMessageFrame(command);
    auto write_buffer = frame.marshallUartPacket();

    int i = 0;
    while (i < MAX_WRITE_ATTEMPTS)
    {
        if (uart->serialWrite(write_buffer))
        {
            uart->flushSerialPort(uart->flush_send);
            break;
        }
        i++;
    }
    if (i == MAX_WRITE_ATTEMPTS)
    {
        LOG(WARNING)
            << "Writing power command failed. Maximum number of attempts exceeded";
    }

    return std::make_unique<PowerStatus>(status);
}
