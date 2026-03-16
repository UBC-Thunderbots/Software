#pragma once

#include "software/embedded/constants/constants.h"
#include "software/embedded/gpio/setup_gpio.hpp"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/embedded/motor_controller/stspin_types.h"

class StSpinMotorController : public MotorController
{
   public:
    explicit StSpinMotorController();

    MotorControllerStatus earlyPoll() override;

    void setup() override;

    void reset() override;

    MotorFaultIndicator checkDriverFault(MotorIndex motor) override;

    int readThenWriteVelocity(MotorIndex motor, int target_velocity) override;

    void immediatelyDisable() override;

   private:
    using OutgoingFrame =
        std::variant<SetResponseTypeFrame, SetTargetSpeedFrame, SetTargetTorqueFrame,
                     SetPidTorqueKpKiFrame, SetPidFluxKpKiFrame, SetPidSpeedKpKiFrame,
                     SetPidSpeedKdFrame>;

    // Length of frame (in number of bytes)
    static constexpr unsigned int FRAME_LEN = 6;

    // clang-format off
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT,  "/dev/spidev0.2"},
        {MotorIndex::BACK_LEFT,   "/dev/spidev0.3"},
        {MotorIndex::BACK_RIGHT,  "/dev/spidev0.0"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.4"},
    };
    // clang-format on

    static constexpr uint32_t SPI_SPEED_HZ     = 100000;
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000;
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    // SPI file descriptors
    std::unordered_map<MotorIndex, int> spi_fds_;

    struct MotorStatus
    {
        bool enabled;
        int16_t measured_speed_rpm;
        uint16_t faults;
        int16_t iq;
        int16_t id;
        int16_t vq;
        int16_t vd;
        int16_t phase_current;
        int16_t phase_voltage;
    };

    std::unordered_map<MotorIndex, MotorStatus> motor_status_;

    std::unique_ptr<Gpio> reset_gpio_;

    /**
     * Opens a SPI file descriptor for the given motor
     *
     * @param motor the motor to open a SPI file descriptor for
     */
    void openSpiFileDescriptor(MotorIndex motor);

    /**
     * Transmits a frame to the given motor and receives a frame back over SPI.
     *
     * @param motor the motor to send the frame to
     * @param outgoing_frame the outgoing frame to send to the motor
     */
    void sendAndReceiveFrame(MotorIndex motor, const OutgoingFrame& outgoing_frame);

    void populateTx(const OutgoingFrame& outgoing_frame,
                    std::array<uint8_t, FRAME_LEN>& tx);

    void processRx(MotorIndex motor, const std::array<uint8_t, FRAME_LEN>& rx);

    friend class StSpinMotorControllerTest;
};
