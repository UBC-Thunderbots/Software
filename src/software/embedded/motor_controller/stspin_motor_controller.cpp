#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <linux/spi/spidev.h>
#include <chrono>
#include <thread>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include "cppcrc.h"
#pragma GCC diagnostic pop

#include "shared/constants.h"
#include "software/embedded/spi_utils.h"
#include "software/logger/logger.h"

// AUTOSAR variant of CRC-8
// (https://reveng.sourceforge.io/crc-catalogue/all.htm#crc.cat.crc-8-autosar)
using Crc8Autosar = crc_utils::crc<uint8_t, 0x2F, 0xFF, false, false, 0xFF>;

StSpinMotorController::StSpinMotorController()
    : reset_gpio_(
          setupGpio(MOTOR_DRIVER_RESET_GPIO, GpioDirection::OUTPUT, GpioState::HIGH))
{
    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        if (ENABLED_MOTORS.at(motor))
        {
            openSpiFileDescriptor(motor);

            data_ready_gpio_[motor] =
                setupGpio(DATA_READY_GPIO_PINS.at(motor), GpioDirection::INPUT,
                          GpioState::LOW);
        }
    }
}

MotorControllerStatus StSpinMotorController::earlyPoll()
{
    // No encoders to calibrate, so just return OK
    return MotorControllerStatus::OK;
}

void StSpinMotorController::setup()
{
    reset();

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        if (ENABLED_MOTORS.at(motor))
        {
            sendAndReceiveFrame(motor, StSpinOpcode::ACK_FAULTS);
        }
    }

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        if (ENABLED_MOTORS.at(motor))
        {
            checkDriverFault(motor);
            readThenWriteVelocity(motor, 0);
            sendAndReceiveFrame(motor, StSpinOpcode::START_MOTOR);
        }
    }
}

void StSpinMotorController::reset()
{
    reset_gpio_->setValue(GpioState::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    reset_gpio_->setValue(GpioState::HIGH);
}

MotorFaultIndicator StSpinMotorController::checkDriverFault(const MotorIndex& motor)
{
    bool drive_enabled = true;
    std::unordered_set<TbotsProto::MotorFault> motor_faults;

    if (!ENABLED_MOTORS.at(motor))
    {
        // Motor is disabled; pretend that the motor is working fine so
        // that Thunderloop doesn't attempt to reset it and crash
        return MotorFaultIndicator(drive_enabled, motor_faults);
    }

    sendAndReceiveFrame(motor, StSpinOpcode::GET_FAULT);

    // We must send a SPI_NOOP in order to clock out the response
    // to the GET_FAULT request.
    const uint16_t faults = sendAndReceiveFrame(motor, StSpinOpcode::SPI_NOOP);

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
    if (!ENABLED_MOTORS.at(motor))
    {
        // Motor is disabled; pretend that the motor is working fine so
        // that Thunderloop doesn't attempt to reset it and crash
        return 0;
    }

    sendAndReceiveFrame(motor, StSpinOpcode::GET_SPEED);

    // SET_SPEEDRAMP expects the target motor speed to be in register ax.
    // Also, the frame we receive here contains the response to the GET_SPEED
    // request made in the previous frame.
    const int16_t current_velocity = sendAndReceiveFrame(
        motor, StSpinOpcode::MOV_AX, static_cast<int16_t>(target_velocity));

    // SET_SPEEDRAMP expects the ramp time in millis to be in register bx.
    // We do speed ramping ourselves in MotorService, so we just want to
    // set the target speed without ramping (hence we set reg bx to 0).
    sendAndReceiveFrame(motor, StSpinOpcode::MOV_BX, 300);

    sendAndReceiveFrame(motor, StSpinOpcode::SET_SPEEDRAMP);

    return current_velocity;
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
    //  A frame is 6 bytes long and has the following format:
    //
    //  +-------+--------+---------------+-------+-------+
    //  |  SOF  | OPCODE |     DATA      |  CRC  |  EOF  |
    //  +-------+--------+---------------+-------+-------+
    //      0       1        2       3       4       5
    //
    //  For slave frames, OPCODE will be either an ACK or NACK acknowledging
    //  receival of the last master frame.
    //
    //  The byte order of DATA is big endian; therefore the MSB is transmitted first.
    //  DATA may be ignored if the operation has no data to transmit/receive.

    // Busy wait for slave data ready assertion
    while (data_ready_gpio_.at(motor)->getValue() != GpioState::HIGH)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    uint8_t tx[FRAME_LEN] = {};
    uint8_t rx[FRAME_LEN] = {};

    tx[0] = FRAME_SOF;
    tx[1] = static_cast<uint8_t>(opcode);
    tx[2] = static_cast<uint8_t>(0xFF & (data >> 8));
    tx[3] = static_cast<uint8_t>(0xFF & data);
    tx[5] = FRAME_EOF;

    const uint8_t tx_msg[] = {tx[1], tx[2], tx[3]};
    tx[4]                  = Crc8Autosar::calc(tx_msg, sizeof(tx_msg));

    spiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], tx, rx, FRAME_LEN,
                SPI_SPEED_HZ);

    // Frame integrity check
    const uint8_t rx_msg[] = {rx[1], rx[2], rx[3]};
    const uint8_t rx_crc   = Crc8Autosar::calc(rx_msg, sizeof(rx_msg));
    if (rx[0] != FRAME_SOF || rx[5] != FRAME_EOF || rx[4] != rx_crc)
    {
        LOG(WARNING) << "Received frame that failed integrity check. Expected CRC "
                     << static_cast<int>(rx_crc) << " but got " << static_cast<int>(rx[4])
                     << " for motor " << motor;
    }

    // Return the DATA field of the received frame
    return static_cast<int16_t>((static_cast<uint16_t>(rx[2]) << 8) | rx[3]);
}
