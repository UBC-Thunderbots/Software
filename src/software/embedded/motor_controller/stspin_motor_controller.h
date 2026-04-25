#pragma once

#include "software/embedded/gpio/gpio.h"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/embedded/motor_controller/stspin_types.h"

class StSpinMotorController : public MotorController
{
   public:
    explicit StSpinMotorController();

    void setup() override;

    void reset() override;

    const MotorFaultIndicator& checkFaults(MotorIndex motor) override;

    int readThenWriteVelocity(MotorIndex motor, int target_velocity) override;

    void immediatelyDisable() override;

   private:
    using OutgoingFrame =
        std::variant<NoOpFrame, SetResponseTypeFrame, SetTargetSpeedFrame,
                     SetTargetTorqueFrame, SetPidTorqueKpKiFrame, SetPidFluxKpKiFrame,
                     SetPidSpeedKpKiFrame, SetSpeedFeedForwardKaKvFrame>;

    // Length of frame (in number of bytes)
    static constexpr unsigned int FRAME_LEN = 6;

    // clang-format off
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT,  "/dev/spidev0.2"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.4"},
        {MotorIndex::BACK_LEFT,   "/dev/spidev0.3"},
        {MotorIndex::BACK_RIGHT,  "/dev/spidev0.0"},
    };
    // clang-format on

    static constexpr uint32_t SPI_SPEED_HZ     = 100000;
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000;
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    static constexpr int RESET_GPIO_PIN = 12;

    // SPI file descriptors
    std::unordered_map<MotorIndex, int> spi_fds_;

    static constexpr uint32_t MAX_CONSECUTIVE_CRC_FAILURES = 10;

    std::unique_ptr<Gpio> reset_gpio_;

    struct MotorStatus
    {
        unsigned int frame_count;
        bool enabled;
        MotorFaultIndicator faults;
        uint16_t fault_flags;
        int16_t speed;
        int16_t speed_ref;
        int16_t iq;
        int16_t iq_ref;
        int16_t id;
        int16_t id_ref;
        int16_t vq;
        int16_t vd;
        int16_t phase_current;
        int16_t phase_voltage;
        uint32_t consecutive_crc_failures = 0;
    };

    std::unordered_map<MotorIndex, MotorStatus> motor_status_;

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

    void updateFaults(MotorIndex motor, uint16_t fault_flags);

    void sendMotorStatusToPlotJuggler(MotorIndex motor);

    friend class StSpinMotorControllerTest;
};
