#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <linux/spi/spidev.h>

#include "shared/constants.h"
#include "software/embedded/spi_utils.h"
#include "software/logger/logger.h"

StSpinMotorController::StSpinMotorController()
    : driver_control_enable_gpio_(
          setupGpio(DRIVER_CONTROL_ENABLE_GPIO, GpioDirection::OUTPUT, GpioState::HIGH)),
      reset_gpio_(
          setupGpio(MOTOR_DRIVER_RESET_GPIO, GpioDirection::OUTPUT, GpioState::HIGH))
{
    openSpiFileDescriptor(MotorIndex::FRONT_LEFT);
    openSpiFileDescriptor(MotorIndex::FRONT_RIGHT);
    openSpiFileDescriptor(MotorIndex::BACK_LEFT);
    openSpiFileDescriptor(MotorIndex::BACK_RIGHT);
    openSpiFileDescriptor(MotorIndex::DRIBBLER);
}

MotorControllerStatus StSpinMotorController::earlyPoll()
{
    // No encoders to calibrate, so just return OK
    return MotorControllerStatus::OK;
}

void StSpinMotorController::setup()
{
    reset_gpio_->setValue(GpioState::LOW);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    reset_gpio_->setValue(GpioState::HIGH);
    usleep(MICROSECONDS_PER_MILLISECOND * 100);

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        sendFrame(motor, StSpinOpcode::ACK_FAULTS);
    }

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        sendFrame(motor, StSpinOpcode::START_MOTOR);
        checkDriverFault(motor);
        readThenWriteVelocity(motor, 0);
    }
}

void StSpinMotorController::reset()
{
    reset_gpio_->setValue(GpioState::LOW);
}

MotorFaultIndicator StSpinMotorController::checkDriverFault(const MotorIndex& motor)
{
    bool drive_enabled = true;
    std::unordered_set<TbotsProto::MotorFault> motor_faults;

    const uint16_t faults = sendAndReceiveFrame(motor, StSpinOpcode::GET_FAULT, 0);

    if (faults != 0)
    {
        LOG(WARNING) << "======= Faults For Motor " << motor << "=======";
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::DURATION))
    {
        LOG(WARNING) << "DURATION: FOC rate too high";
        motor_faults.insert(TbotsProto::MotorFault::DURATION);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVER_VOLT))
    {
        LOG(WARNING) << "OVER_VOLT: Software overvoltage";
        motor_faults.insert(TbotsProto::MotorFault::OVER_VOLT);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::UNDER_VOLT))
    {
        LOG(WARNING) << "UNDER_VOLT: Software undervoltage";
        motor_faults.insert(TbotsProto::MotorFault::UNDER_VOLT);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVER_TEMP))
    {
        LOG(WARNING) << "OVER_TEMP: Software over temperature";
        motor_faults.insert(TbotsProto::MotorFault::OVER_TEMP);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::START_UP))
    {
        LOG(WARNING) << "START_UP: Start up failed";
        motor_faults.insert(TbotsProto::MotorFault::START_UP);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::SPEED_FDBK))
    {
        LOG(WARNING) << "SPEED_FDBK: Speed feedback fault";
        motor_faults.insert(TbotsProto::MotorFault::SPEED_FDBK);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVER_CURR))
    {
        LOG(WARNING) << "OVER_CURR: Software overcurrent";
        motor_faults.insert(TbotsProto::MotorFault::OVER_CURR);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::SW_ERROR))
    {
        LOG(WARNING) << "SW_ERROR: Software error";
        motor_faults.insert(TbotsProto::MotorFault::SW_ERROR);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::SAMPLE_FAULT))
    {
        LOG(WARNING) << "SAMPLE_FAULT: Sample fault for testing purposes";
        motor_faults.insert(TbotsProto::MotorFault::SAMPLE_FAULT);
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVERCURR_SW))
    {
        LOG(INFO) << "OVERCURR_SW: Software overcurrent";
        motor_faults.insert(TbotsProto::MotorFault::OVERCURR_SW);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::DP_FAULT))
    {
        LOG(WARNING) << "DP_FAULT: Driver protection fault";
        motor_faults.insert(TbotsProto::MotorFault::DP_FAULT);
        drive_enabled = false;
    }

    return MotorFaultIndicator(drive_enabled, motor_faults);
}

double StSpinMotorController::readThenWriteVelocity(const MotorIndex& motor,
                                                    const int& target_velocity)
{
    const int16_t speed = sendAndReceiveFrame(motor, StSpinOpcode::GET_SPEED, 0);

    // SET_SPEEDRAMP expects the target motor speed to be in register ax.
    sendAndReceiveFrame(motor, StSpinOpcode::MOV_AX,
                        static_cast<int16_t>(target_velocity));

    // SET_SPEEDRAMP expects the ramp time in millis to be in register bx.
    // We do speed ramping ourselves in MotorService, so we just want to
    // set the target speed without ramping (hence we set reg bx to 0).
    sendAndReceiveFrame(motor, StSpinOpcode::MOV_BX, 0);

    sendFrame(motor, StSpinOpcode::SET_SPEEDRAMP);

    return speed;
}

void StSpinMotorController::immediatelyDisable()
{
    driver_control_enable_gpio_->setValue(GpioState::LOW);
}

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
    tx[2] = static_cast<uint8_t>(0xFF & (data >> 8));
    tx[3] = static_cast<uint8_t>(0xFF & data);
    tx[4] = 0;  // CRC; to be implemented later
    tx[5] = FRAME_EOF;

    spiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], tx, rx, FRAME_MAX_LEN,
                SPI_SPEED_HZ);

    return static_cast<int16_t>((static_cast<uint16_t>(rx[2]) << 8) | rx[3]);
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
