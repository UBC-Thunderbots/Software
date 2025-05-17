#pragma once

#include "software/embedded/motor_controller/motor_controller.h"
#include "software/util/make_enum/make_enum.hpp"

class StSpinMotorController : public MotorController
{
   public:
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
     * @param motor the motor to send the frame to
     * @param opcode the opcode to include in the transmitted frame
     * @param data the data to include in the transmitted frame
     * @return the data in the frame received from the motor
     */
    int16_t sendAndReceiveFrame(const MotorIndex& motor, const StSpinOpcode opcode,
                                const int16_t data);

    /**
     * Transmits a frame to the given motor over SPI without expecting any data
     * back in response.
     *
     * @param motor the motor to send the frame to
     * @param opcode the opcode to include in the transmitted frame
     */
    void sendFrame(const MotorIndex& motor, const StSpinOpcode opcode);

    enum class StSpinOpcode
    {
        SPI_NOOP      = 0b00000000,
        MOV_AX        = 0b10000010,
        GET_AX        = 0b10000011,
        MOV_BX        = 0b10000100,
        GET_BX        = 0b10000101,
        SET_SPEEDRAMP = 0b00000010,
        GET_SPEED     = 0b00000011,
        SET_ENCODER   = 0b00000100,
        GET_ENCODER   = 0b00000101,
        START_MOTOR   = 0b00001000,
        STOP_MOTOR    = 0b11111111,
        ACK_FAULTS    = 0b00010000,
        GET_FAULT     = 0b00010001,
        SET_CURRENT   = 0b00100000,
        GET_CURRENT   = 0b00100001,
        ACK           = 0b11000000,
        NACK          = 0b11000001,
        SPI_ERROR     = 0b11100000
    };

    enum class StSpinFaultCode
    {
        NO_FAULT     = 0x0000,
        DURATION     = 0x0001,
        OVER_VOLT    = 0x0002,
        UNDER_VOLT   = 0x0004,
        OVER_TEMP    = 0x0008,
        START_UP     = 0x0010,
        SPEED_FDBK   = 0x0020,
        OVER_CURR    = 0x0040,
        SW_ERROR     = 0x0080,
        SAMPLE_FAULT = 0x0100,
        OVERCURR_SW  = 0x0200,
        DP_FAULT     = 0x0400
    };

    // Start-of-frame and end-of-frame markers
    static constexpr uint8_t FRAME_SOF = 0x73;
    static constexpr uint8_t FRAME_EOF = 0x45;

    // Length of frame (in number of bytes)
    static constexpr unsigned int FRAME_MIN_LEN = 4;
    static constexpr unsigned int FRAME_MAX_LEN = FRAME_MIN_LEN + 2;

    // SPI Chip Selects
    static constexpr uint8_t FRONT_LEFT_MOTOR_CHIP_SELECT  = 0;
    static constexpr uint8_t BACK_LEFT_MOTOR_CHIP_SELECT   = 1;
    static constexpr uint8_t BACK_RIGHT_MOTOR_CHIP_SELECT  = 2;
    static constexpr uint8_t FRONT_RIGHT_MOTOR_CHIP_SELECT = 3;
    static constexpr uint8_t DRIBBLER_MOTOR_CHIP_SELECT    = 4;

    static const inline std::unordered_map<MotorIndex, uint8_t> CHIP_SELECTS = {
        {MotorIndex::FRONT_LEFT, FRONT_LEFT_MOTOR_CHIP_SELECT},
        {MotorIndex::BACK_LEFT, BACK_LEFT_MOTOR_CHIP_SELECT},
        {MotorIndex::BACK_RIGHT, BACK_RIGHT_MOTOR_CHIP_SELECT},
        {MotorIndex::FRONT_RIGHT, FRONT_RIGHT_MOTOR_CHIP_SELECT},
        {MotorIndex::DRIBBLER, DRIBBLER_MOTOR_CHIP_SELECT},
    };

    // SPI Motor Driver Paths
    static const inline std::unordered_map<MotorIndex, const char*> SPI_PATHS = {
        {MotorIndex::FRONT_LEFT, "/dev/spidev0.0"},
        {MotorIndex::FRONT_RIGHT, "/dev/spidev0.1"},
        {MotorIndex::BACK_LEFT, "/dev/spidev0.2"},
        {MotorIndex::BACK_RIGHT, "/dev/spidev0.3"},
        {MotorIndex::DRIBBLER, "/dev/spidev0.4"},
    };

    // SPI Configs
    static constexpr uint32_t SPI_SPEED_HZ     = 8000000;  // 8 Mhz
    static constexpr uint32_t MAX_SPI_SPEED_HZ = 8000000;  // 8 Mhz
    static constexpr uint8_t SPI_BITS          = 8;
    static constexpr uint32_t SPI_MODE         = 0;

    // SPI File Descriptors mapping from Chip Select -> File Descriptor
    std::array<int, reflective_enum::size<MotorIndex>()> file_descriptors_;
};
