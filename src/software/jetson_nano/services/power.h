#pragma once

#include <atomic>
#include <thread>

#include "proto/power_frame_msg.pb.h"
#include "shared/uart_framing/uart_framing.hpp"
#include "software/logger/logger.h"
#include "software/uart/boost_uart_communication.h"

extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}

class PowerService
{
   public:
    /**
     * Service that interacts with the power board.
     * Opens all the required ports and maintains them until destroyed.
     */
    PowerService();
    ~PowerService();

    /**
     * When the power service is polled it sends the given power control msg and
     * returns the latest power status
     *
     * @param control The power control msg to send
     * @return the latest power status
     */
    TbotsProto::PowerStatus poll(const TbotsProto::PowerControl& control,
                                 double kick_coeff, int kick_constant, 
                                 double chip_coeff, int chip_constant);

    /**
     * Handler method called every time the timer expires a new read is requested
     */
    void tick();

   private:
    /**
     * Initiates timer for serial reading
     */
    void continuousRead();

    std::thread read_thread;
    std::atomic<TbotsProto_PowerStatus> status;
    std::atomic<TbotsProto_PowerPulseControl> nanopb_command;
    std::unique_ptr<BoostUartCommunication> uart;

    // Constants
    const size_t READ_BUFFER_SIZE =
        getMarshalledSize(TbotsProto_PowerStatus TbotsProto_PowerStatus_init_default);
    const std::string DEVICE_SERIAL_PORT    = "/dev/ttyUSB0";
    static constexpr unsigned int BAUD_RATE = 460800;
};
