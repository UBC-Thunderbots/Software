#pragma once

#include <atomic>
#include <thread>

#include "proto/power_frame_msg.pb.h"
#include "shared/uart_framing/uart_framing.hpp"
#include "software/logger/logger.h"
#include "software/uart/boost_uart_communication.h"

extern "C"
{
#include "proto/power_frame_msg.pb.h"
}

/**
 * Communicates to the powerboard over uart.
 */
class UartCommunicator
{
   public:
    /**
     * Service that interacts with the power board.
     * Opens all the required ports and maintains them until destroyed.
     */
    UartCommunicator();
    ~UartCommunicator();

    /**
     * When the power service is polled it sends the given power control msg and
     * returns the latest power status
     *
     * @param control The power control msg to send
     * @return the latest power status
     */
    TbotsProto::PowerStatus sendChipKickCommand(const TbotsProto::PowerControl& control,
                                                double kick_coeff, int kick_constant,
                                                int chip_constant);

    /**
     * Set dribbler RPM
     */
    void sendDribbleTarget(int rpm);

    /**
     * Handler method called every time the timer expires a new read is requested
     */
    virtual void tick();

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
    const std::string DEVICE_SERIAL_PORT    = "/dev/ttyAMA0";
    static constexpr unsigned int BAUD_RATE = 460800;

    // Required flag to exit power service cleanly
    bool is_running = true;

    bool _new_dribble_command = false;
    std::atomic<TbotsProto_DribblerControl> dribble_command;

    int prev_commanded_dribble_rpm = 0;
};
