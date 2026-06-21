#pragma once

#include <chrono>

#include "software/embedded/gpio/gpio.h"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/embedded/motor_controller/stspin_types.h"

/**
 * Motor controller for controlling our 6th generation STSPIN motor drivers.
 *
 * We communicate with our MDv6 boards over SPI in full-duplex mode using a custom
 * frame-based protocol. This protocol is documented in the MDv6 firmware repo, which
 * can be found at https://github.com/UBC-Thunderbots/MDv6_Firmware
 */
class StSpinMotorController : public MotorController
{
   public:
    explicit StSpinMotorController(
        const robot_constants::RobotConstants& robot_constants);

    void setup() override;

    void reset() override;

    const MotorFaultIndicator& checkFaults(MotorIndex motor) override;

    int readThenWriteVelocity(MotorIndex motor, int target_velocity) override;

    void immediatelyDisable() override;

   private:
    // TODO: #3750 Use a template function instead of std::variant.
    using OutgoingMessage =
        std::variant<NoOpMessage, SetResponseTypeMessage, SetTargetSpeedMessage,
                     SetTargetTorqueMessage, SetPidTorqueKpKiMessage,
                     SetPidFluxKpKiMessage, SetPidSpeedKpKiMessage,
                     SetSpeedFeedForwardKaKvMessage, SetSpeedFeedForwardKsMessage>;

    // Length of message (in number of bytes)
    static constexpr unsigned int MESSAGE_SIZE = 8;

    // Delimiter byte used to indicate start of a message
    static constexpr uint8_t MESSAGE_DELIMITER = 0xAA;

    // Maximum number of SPI transfer attempts to wait for an acknowledgement
    // before giving up. Prevents unresponsive MD (e.g. firmware crash or
    // SPI desynced) from blocking Thunderloop indefinitely
    static constexpr unsigned int MAX_SPI_TRANSFER_ATTEMPTS = 100;

    // clang-format off
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT,  "/dev/spidev0.3"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.2"},
        {MotorIndex::BACK_LEFT,   "/dev/spidev0.1"},
        {MotorIndex::BACK_RIGHT,  "/dev/spidev0.0"},
    };
    // clang-format on

    static constexpr uint32_t SPI_SPEED_HZ     = 500000;
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000;
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    static constexpr int RESET_GPIO_PIN = 12;

    static constexpr int SPEED_PID_PROPORTIONAL_GAIN  = 439;
    static constexpr int SPEED_PID_INTEGRAL_GAIN      = 535;
    static constexpr int TORQUE_PID_PROPORTIONAL_GAIN = 336;
    static constexpr int TORQUE_PID_INTEGRAL_GAIN     = 170;

    robot_constants::RobotConstants robot_constants_;

    // SPI file descriptors
    std::unordered_map<MotorIndex, int> spi_fds_;

    std::unique_ptr<Gpio> reset_gpio_;

    struct MotorStatus
    {
        uint8_t seq_num;
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
        std::chrono::steady_clock::time_point last_message_ack_time;
    };

    std::unordered_map<MotorIndex, MotorStatus> motor_status_;

    /**
     * Opens a SPI file descriptor for the given motor.
     *
     * @param motor the motor to open a SPI file descriptor for
     */
    void openSpiFileDescriptor(MotorIndex motor);

    /**
     * Transmits a message to the given motor and receives a message back over SPI.
     *
     * @param motor the motor to send the message to
     * @param outgoing_message the outgoing message to send to the motor
     */
    void sendAndReceiveMessage(MotorIndex motor, const OutgoingMessage& outgoing_message);

    /**
     * Populates the transmit buffer with the data from an outgoing message.
     *
     * @param motor the motor to send the message to
     * @param outgoing_message the outgoing message to populate the transmit buffer with
     * @param tx the transmit buffer to populate with the outgoing message's data
     */
    void populateTx(MotorIndex motor, const OutgoingMessage& outgoing_message,
                    std::array<uint8_t, MESSAGE_SIZE>& tx);

    /**
     * Processes a message received from the given motor, caching any motor status
     * the message provides.
     *
     * @param motor the motor that the received message corresponds to
     * @param message the received message to process
     */
    void processRx(MotorIndex motor, const std::array<uint8_t, MESSAGE_SIZE>& message);

    /**
     * Records that the given faults were raised for a given motor.
     *
     * @param motor the motor to update the faults of
     * @param fault_flags the faults for the motor
     */
    void updateFaults(MotorIndex motor, uint16_t fault_flags);

    /**
     * Sends the target and measured speed and current (Iq and Id) for the
     * specified motor to PlotJuggler.
     *
     * @param motor the motor to plot the status of
     */
    void sendMotorStatusToPlotJuggler(MotorIndex motor);

    friend class StSpinMotorControllerTest;
};
