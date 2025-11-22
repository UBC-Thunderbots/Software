#pragma once

#include "software/embedded/constants/constants.h"
#include "software/embedded/gpio/setup_gpio.hpp"
#include "software/embedded/motor_controller/motor_controller.h"
#include "software/embedded/motor_controller/motor_fault_indicator.h"
#include "software/embedded/motor_controller/motor_index.h"
#include "software/embedded/motor_controller/stspin_constants.h"

class StSpinMotorController : public MotorController
{
   public:
    explicit StSpinMotorController();

    MotorControllerStatus earlyPoll() override;

    void setup() override;

    void reset() override;

    MotorFaultIndicator checkDriverFault(const MotorIndex& motor) override;

    double readThenWriteVelocity(const MotorIndex& motor,
                                 const int& target_velocity) override;

    void immediatelyDisable() override;

    /**
     * Opens a SPI file descriptor for the given motor
     *
     * @param motor the motor to open a SPI file descriptor for
     */
    void openSpiFileDescriptor(const MotorIndex& motor);

    /**
     * Transmits a frame to the given motor and receives a frame back over SPI.
     *
     * Note: to receive data for GET operations, it is expected that we send a
     * second frame (e.g. a NOOP) after the initial frame requesting the GET.
     * The data in the second frame received will contain the response to the
     * GET request.
     *
     * @param motor the motor to send the frame to
     * @param opcode the opcode to include in the transmitted frame
     * @param data the data to include in the transmitted frame
     * @return the data in the frame received from the motor
     */
    int16_t sendAndReceiveFrame(const MotorIndex& motor, const StSpinOpcode opcode,
                                const int16_t data = 0);

    // Start-of-frame and end-of-frame markers
    static constexpr uint8_t FRAME_SOF = 0x73;
    static constexpr uint8_t FRAME_EOF = 0x45;

    // Length of frame (in number of bytes)
    static constexpr unsigned int FRAME_LEN = 6;

    // SPI Chip Selects
    static constexpr uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static constexpr uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
    static constexpr uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;
    static constexpr uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
    static constexpr uint8_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;

    static const inline std::unordered_map<MotorIndex, bool> ENABLED_MOTORS = {
        {MotorIndex::FRONT_LEFT, true},
        {MotorIndex::BACK_LEFT, true},
        {MotorIndex::BACK_RIGHT, true},
        {MotorIndex::FRONT_RIGHT, true},
        {MotorIndex::DRIBBLER, false},
    };

    static const inline std::unordered_map<MotorIndex, uint8_t> CHIP_SELECTS = {
        {MotorIndex::FRONT_LEFT, FRONT_LEFT_MOTOR_CHIP_SELECT},
        {MotorIndex::BACK_LEFT, BACK_LEFT_MOTOR_CHIP_SELECT},
        {MotorIndex::BACK_RIGHT, BACK_RIGHT_MOTOR_CHIP_SELECT},
        {MotorIndex::FRONT_RIGHT, FRONT_RIGHT_MOTOR_CHIP_SELECT},
        {MotorIndex::DRIBBLER, DRIBBLER_MOTOR_CHIP_SELECT},
    };

    // SPI Motor Driver Paths
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT, "/dev/spidev0.2"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.4"},
        {MotorIndex::BACK_LEFT, "/dev/spidev0.3"},
        {MotorIndex::BACK_RIGHT, "/dev/spidev0.0"},
        {MotorIndex::DRIBBLER, "/dev/spidev0.1"},
    };

    // SPI Configs
    static constexpr uint32_t SPI_SPEED_HZ     = 2000000;   // 2 MHz
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000; // 250 MHz
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    // SPI File Descriptors mapping from Chip Select -> File Descriptor
    std::array<int, reflective_enum::size<MotorIndex>()> file_descriptors_;

    std::unique_ptr<Gpio> reset_gpio_;
    std::unique_ptr<Gpio> data_ready_gpio_;
};
