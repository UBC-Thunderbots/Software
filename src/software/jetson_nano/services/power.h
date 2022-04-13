#pragma once

#include <boost/asio.hpp>

#include "proto/power_frame_msg.pb.h"
#include "software/jetson_nano/services/service.h"
#include "software/logger/logger.h"
#include "software/uart/boost_uart_communication.h"
extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}

class PowerService : public Service
{
   public:
    /**
     * Service that interacts with the power board.
     * Opens all the required ports and maintains them until destroyed.
     */
    PowerService();
    ~PowerService() = default;
    void start() override;
    void stop() override;
    /**
     * When the power service is polled it sends the given power control msg and
     * returns the latest power status
     *
     * @param control The power control msg to send
     * @return the latest power status
     */
    std::unique_ptr<TbotsProto::PowerStatus> poll(
        const TbotsProto::PowerControl& control);

   private:
    boost::asio::io_service io_service;
    std::unique_ptr<BoostUartCommunication> uart;
    TbotsProto_PowerStatus status;

    // Constants
    const size_t READ_BUFFER_SIZE;
    const unsigned int BAUD_RATE         = 115200;
    const std::string DEVICE_SERIAL_PORT = "/dev/ttyTHS1";
};
