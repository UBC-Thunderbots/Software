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
     *
     * @param kick_coefficient The coefficient used in kick speed to pulse width
     * conversion
     * @param kick_constant The constant used in kick speed to pulse width conversion
     * @param chip_constant The constant used in chip distance to pulse width conversion
     */
    explicit PowerService(double kick_coefficient, int kick_constant, int chip_constant);
    ~PowerService();

    /**
     * Polls the power service to execute the given DirectControlPrimitive and update
     * the current power status.
     *
     * @param primitive DirectControlPrimitive to execute
     * @param robot_status RobotStatus message to modify with the current power status
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

    /**
     * Reads a power status frame from the power board over UART.
     *
     * @return the power status read from the power board, or nullopt if the
     * read was unsuccessful
     */
    std::optional<TbotsProto_PowerStatus> readPowerStatus() const;

    /**
     * Writes a power frame to the power board over UART.
     *
     * @param frame the power frame to write to the power board
     */
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
    const std::string DEVICE_SERIAL_PORT    = "/dev/ttyAMA0";
    static constexpr unsigned int BAUD_RATE = 460800;

    // Required flag to exit power service cleanly
    bool is_running_ = true;
};
