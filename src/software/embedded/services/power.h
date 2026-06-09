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

class PowerService
{
   public:
    /**
     * Service that interacts with the power board.
     * Opens all the required ports and maintains them until destroyed.
     */
    explicit PowerService(double kick_coefficient, int kick_constant, int chip_constant);
    ~PowerService();

    /**
     * When the power service is polled it sends the given power control msg and
     * returns the latest power status
     *
     * @return the latest power status
     */
    void poll(const TbotsProto::DirectControlPrimitive& primitive,
              TbotsProto::RobotStatus& robot_status);

    /**
     * Handler method called every time the timer expires a new read is requested
     */
    void tick();

   private:
    /**
     * Initiates timer for serial reading
     */
    void continuousRead();

    std::optional<TbotsProto_PowerStatus> readPowerStatus() const;

    void writePowerFrame(const TbotsProto_PowerFrame& frame) const;

    const double kick_coefficient_;
    const int kick_constant_;
    const int chip_constant_;

    std::thread read_thread_;
    std::atomic<TbotsProto_PowerStatus> power_status_;
    std::atomic<TbotsProto_PowerPulseControl> power_pulse_command_;
    std::atomic<TbotsProto_DribblerControl> dribbler_command_;
    std::unique_ptr<BoostUartCommunication> uart_;

    // Constants
    const size_t READ_BUFFER_SIZE =
        getMarshalledSize(TbotsProto_PowerStatus TbotsProto_PowerStatus_init_default);
    const std::string DEVICE_SERIAL_PORT    = "/dev/ttyUSB0";
    static constexpr unsigned int BAUD_RATE = 460800;

    // Required flag to exit power service cleanly
    bool is_running_ = true;
};
