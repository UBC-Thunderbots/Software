#include "software/embedded/motor_controller/stspin_motor_controller.h"

StSpinMotorController::StSpinMotorController()
{
    openSpiFileDescriptor(MotorIndex::FRONT_LEFT);
    openSpiFileDescriptor(MotorIndex::FRONT_RIGHT);
    openSpiFileDescriptor(MotorIndex::BACK_LEFT);
    openSpiFileDescriptor(MotorIndex::BACK_RIGHT);
    openSpiFileDescriptor(MotorIndex::DRIBBLER);
}

MotorControllerStatus StSpinMotorController::earlyPoll()
{
    return MotorControllerStatus::OK;
}

void StSpinMotorController::setup()
{
    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        sendFrame(motor, StSpinOpcode.START_MOTOR);
    }
}

void StSpinMotorController::reset() {}

MotorFaultIndicator StSpinMotorController::checkDriverFault(const MotorIndex& motor)
{
    return MotorFaultIndicator();
}

double StSpinMotorController::readThenWriteVelocity(const MotorIndex& motor,
                                                    const int& target_velocity)
{
    const int16_t speed = sendAndReceiveFrame(motor, StSpinOpcode.GET_SPEED, 0);

    // SET_SPEEDRAMP expects the target motor speed to be in register ax.
    sendAndReceiveFrame(motor, StSpinOpcode.MOV_AX, target_velocity);

    // SET_SPEEDRAMP expects the ramp time in millis to be in register bx.
    // We do speed ramping ourselves in MotorService, so we just want to
    // set the target speed without ramping (hence we set reg bx to 0).
    sendAndReceiveFrame(motor, StSpinOpcode.MOV_BX, 0);

    sendFrame(motor, StSpinOpcode.SET_SPEEDRAMP);

    return speed;
}

void StSpinMotorController::immediatelyDisable() {}

void StSpinMotorController::openSpiFileDescriptor(const MotorIndex& motor)
{
    file_descriptors_[CHIP_SELECTS.at(motor)] = open(SPI_PATHS.at(motor), O_RDWR);
    CHECK(file_descriptors_[CHIP_SELECTS.at(motor)] >= 0)
        << "can't open device: " << motor << "error: " << strerror(errno);

    int ret =
        ioctl(file_descriptors_[CHIP_SELECTS.at(motor)], SPI_IOC_WR_MODE32, &SPI_MODE);
    CHECK(ret != -1) << "can't set spi mode for: " << motor
                     << "error: " << strerror(errno);

    ret = ioctl(file_descriptors_[CHIP_SELECTS.at(motor)], SPI_IOC_WR_BITS_PER_WORD,
                &SPI_BITS);
    CHECK(ret != -1) << "can't set bits_per_word for: " << motor
                     << "error: " << strerror(errno);

    ret = ioctl(file_descriptors_[CHIP_SELECTS.at(motor)], SPI_IOC_WR_MAX_SPEED_HZ,
                &MAX_SPI_SPEED_HZ);
    CHECK(ret != -1) << "can't set spi max speed hz for: " << motor
                     << "error: " << strerror(errno);
}

int16_t StSpinMotorController::sendAndReceiveFrame(const MotorIndex& motor,
                                                   const StSpinOpcode opcode,
                                                   const int16_t data)
{
    //  A full frame is 6 bytes long and has the following format:
    //
    //  +-------+--------+---------------+-------+-------+
    //  |  SOF  | OPCODE |     DATA      |  CRC  |  EOF  |
    //  +-------+--------+---------------+-------+-------+
    //      0       1        2       3       4       5
    //
    //  For slave frames, OPCODE will be either an ACK or NACK acknowledging
    //  receival of the master frame.
    //
    //  The byte order of DATA is big endian; therefore the MSB is transmitted first.
    //  DATA may be omitted if the operation has no data to transmit/receive, making
    //  the frame only 4 bytes long.
    //
    //  To receive data for GET operations, it is expected that we send a master frame
    //  with 2 dummy DATA bytes in order to clock out the 2 DATA bytes from the incoming
    //  slave frame.

    uint8_t tx[FRAME_MAX_LEN] = {0};
    uint8_t rx[FRAME_MAX_LEN] = {0};

    tx[0] = FRAME_SOF;
    tx[1] = static_cast<uint8_t>(opcode);
    tx[2] = 0xFF & (data >> 8);
    tx[3] = 0xFF & data;
    tx[4] = 0;  // CRC; to be implemented later
    tx[5] = FRAME_EOF;

    spiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], tx, rx, FRAME_MAX_LEN,
                SPI_SPEED_HZ);

    return static_cast<int16_t>(static_cast<uint16_t>(rx[2]) << 8) | rx[3]);
}

void StSpinMotorController::sendFrame(const MotorIndex& motor, const StSpinOpcode opcode)
{
    uint8_t tx[FRAME_MIN_LEN] = {0};
    uint8_t rx[FRAME_MIN_LEN] = {0};

    tx[0] = FRAME_SOF;
    tx[1] = static_cast<uint8_t>(opcode);
    tx[2] = 0;  // CRC; to be implemented later
    tx[3] = FRAME_EOF;

    spiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], tx, rx, FRAME_MIN_LEN,
                SPI_SPEED_HZ);
}
