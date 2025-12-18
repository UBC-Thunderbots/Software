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

   private:
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

    static const inline std::unordered_map<MotorIndex, bool> ENABLED_MOTORS = {
        {MotorIndex::FRONT_LEFT,  false},
        {MotorIndex::BACK_LEFT,   false},
        {MotorIndex::BACK_RIGHT,  false},
        {MotorIndex::FRONT_RIGHT, true},
        {MotorIndex::DRIBBLER,    false},
    };

    // SPI Chip Selects
    // clang-format off
    static const inline std::unordered_map<MotorIndex, uint8_t> CHIP_SELECTS = {
        {MotorIndex::FRONT_LEFT,  0},
        {MotorIndex::BACK_LEFT,   1},
        {MotorIndex::BACK_RIGHT,  2},
        {MotorIndex::FRONT_RIGHT, 3},
        {MotorIndex::DRIBBLER,    4},
    };
    // clang-format on

    // SPI Motor Driver Paths
    // clang-format off
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT,  "/dev/spidev0.2"},
        {MotorIndex::BACK_LEFT,   "/dev/spidev0.3"},
        {MotorIndex::BACK_RIGHT,  "/dev/spidev0.0"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.4"},
        {MotorIndex::DRIBBLER,    "/dev/spidev0.1"},
    };
    // clang-format on

    // Data Ready GPIO Pins
    // clang-format off
    static const inline std::unordered_map<MotorIndex, uint8_t> DATA_READY_GPIO_PINS = {
        {MotorIndex::FRONT_LEFT,  24},
        {MotorIndex::BACK_LEFT,   16},
        {MotorIndex::BACK_RIGHT,  26},
        {MotorIndex::FRONT_RIGHT, 23},
        {MotorIndex::DRIBBLER,    0},
    };
    // clang-format on

    // SPI Configs
    static constexpr uint32_t SPI_SPEED_HZ     = 100000;    // 100 KHz
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 250000000; // 250 MHz
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    // SPI File Descriptors mapping from Chip Select -> File Descriptor
    std::array<int, reflective_enum::size<MotorIndex>()> file_descriptors_;

    std::unique_ptr<Gpio> reset_gpio_;
    std::unordered_map<MotorIndex, std::unique_ptr<Gpio>> data_ready_gpio_;
};
