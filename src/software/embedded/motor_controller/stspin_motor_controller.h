#pragma once

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

    void updateEuclideanVelocity(EuclideanSpace_t target_euclidean_velocity) override;

    void immediatelyDisable() override;

   private:
    // TODO: #3750 Use a template function instead of std::variant.
    using OutgoingFrame =
        std::variant<NoOpFrame, SetResponseTypeFrame, SetTargetSpeedFrame,
                     SetTargetTorqueFrame, SetPidTorqueKpKiFrame, SetPidFluxKpKiFrame,
                     SetPidSpeedKpKiFrame, SetSpeedFeedForwardKaKvFrame,
                     SetSpeedFeedForwardKsFrame>;

    // Length of frame (in number of bytes)
    static constexpr unsigned int FRAME_LEN = 6;

    // clang-format off
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT,  "/dev/spidev0.3"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.2"},
        {MotorIndex::BACK_LEFT,   "/dev/spidev0.1"},
        {MotorIndex::BACK_RIGHT,  "/dev/spidev0.0"},
    };
    // clang-format on

    static constexpr uint32_t SPI_SPEED_HZ     = 100000;
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000;
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    static constexpr int RESET_GPIO_PIN = 12;

    static constexpr int SPEED_PID_PROPORTIONAL_GAIN = 700;
    static constexpr int SPEED_PID_INTEGRAL_GAIN     = 30;

    static constexpr int MAX_SPEED_FEED_FORWARD_STATIC_GAIN = 750;
    static constexpr int MIN_SPEED_FEED_FORWARD_STATIC_GAIN = 300;
    static constexpr double MINIMUM_SPEED_FOR_FEED_FORWARD  = 0.01;

    robot_constants::RobotConstants robot_constants_;

    // SPI file descriptors
    std::unordered_map<MotorIndex, int> spi_fds_;

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
    };

    std::unordered_map<MotorIndex, MotorStatus> motor_status_;

    /**
     * Opens a SPI file descriptor for the given motor.
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

    /**
     * Populates the transmit buffer with the data from an outgoing frame.
     *
     * @param outgoing_frame the outgoing frame to populate the transmit buffer with
     * @param tx the transmit buffer to populate with the outgoing frame's data
     */
    void populateTx(const OutgoingFrame& outgoing_frame,
                    std::array<uint8_t, FRAME_LEN>& tx);

    /**
     * Processes a frame received from the given motor, caching any motor status
     * the frame provides.
     *
     * @param motor the motor that the received frame corresponds to
     * @param rx the receive buffer with the received frame to process
     */
    void processRx(MotorIndex motor, const std::array<uint8_t, FRAME_LEN>& rx);

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
