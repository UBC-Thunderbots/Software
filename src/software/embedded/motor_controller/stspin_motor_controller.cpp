#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <linux/spi/spidev.h>

#include <chrono>
#include <thread>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include "cppcrc.h"
#pragma GCC diagnostic pop

#include "shared/constants.h"
#include "software/embedded/motor_controller/stspin_constants.h"
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
            motor_enabled_[motor]            = true;
            motor_measured_speed_rpm_[motor] = 0;
            motor_faults_[motor]             = 0;
            motor_iq_[motor]                 = 0;
            motor_id_[motor]                 = 0;
        }
    }
}

void StSpinMotorController::reset()
{
    reset_gpio_->setValue(GpioState::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    reset_gpio_->setValue(GpioState::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    const uint16_t faults = motor_faults_.at(motor);

    if (faults == 0)
    {
        // No faults; early return
        return MotorFaultIndicator(drive_enabled, motor_faults);
    }

    LOG(WARNING) << "======= Faults For Motor " << motor << "=======";

    if (faults & static_cast<uint16_t>(StSpinFaultCode::DURATION))
    {
        LOG(WARNING) << "DURATION: FOC rate too high";
        motor_faults.insert(TbotsProto::MotorFault::DURATION);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVER_VOLT))
    {
        LOG(WARNING) << "OVER_VOLT: Over voltage";
        motor_faults.insert(TbotsProto::MotorFault::OVER_VOLT);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::UNDER_VOLT))
    {
        LOG(WARNING) << "UNDER_VOLT: Under voltage";
        motor_faults.insert(TbotsProto::MotorFault::UNDER_VOLT);
        drive_enabled = false;
    }

    if (faults & static_cast<uint16_t>(StSpinFaultCode::OVER_TEMP))
    {
        LOG(WARNING) << "OVER_TEMP: Over temperature";
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
        LOG(WARNING) << "OVER_CURR: Over current";
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
        LOG(INFO) << "OVERCURR_SW: Software over current";
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

int StSpinMotorController::readThenWriteVelocity(const MotorIndex& motor,
                                                 const int& target_velocity)
{
    if (!ENABLED_MOTORS.at(motor))
    {
        // Motor is disabled; pretend that the motor is working fine so
        // that Thunderloop doesn't attempt to reset it and crash
        return 0;
    }

    const auto outgoing_frame = SetTargetSpeedFrame{
        .motor_enabled          = motor_enabled_.at(motor),
        .motor_target_speed_rpm = static_cast<int16_t>(target_velocity),
    };

    sendAndReceiveFrame(motor, outgoing_frame);

    return motor_measured_speed_rpm_.at(motor);
}

void StSpinMotorController::immediatelyDisable()
{
    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        if (ENABLED_MOTORS.at(motor))
        {
            motor_enabled_[motor] = false;
            readThenWriteVelocity(motor, 0);
        }
    }
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

void StSpinMotorController::sendAndReceiveFrame(const MotorIndex& motor,
                                                const OutgoingFrame outgoing_frame)
{
    uint8_t tx[FRAME_LEN] = {};
    uint8_t rx[FRAME_LEN] = {};

    std::visit(
        [&]<typename TFrame>(TFrame&& frame)
        {
            using T = std::decay_t<TFrame>;
            if constexpr (std::is_same_v<T, SetTargetSpeedFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_TARGET_SPEED);
                tx[1] = static_cast<uint8_t>(frame.motor_enabled);
                tx[2] = static_cast<uint8_t>(0xFF & (frame.motor_target_speed_rpm >> 8));
                tx[3] = static_cast<uint8_t>(0xFF & frame.motor_target_speed_rpm);
            }
            else if constexpr (std::is_same_v<T, SetTargetTorqueFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_TARGET_TORQUE);
                tx[1] = static_cast<uint8_t>(frame.motor_enabled);
                tx[2] = static_cast<uint8_t>(0xFF & (frame.motor_target_torque >> 8));
                tx[3] = static_cast<uint8_t>(0xFF & frame.motor_target_torque);
            }
            else if constexpr (std::is_same_v<T, SetResponseTypeFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_RESPONSE_TYPE);
                tx[1] = static_cast<uint8_t>(frame.response_type);
            }
            else if constexpr (std::is_same_v<T, SetPidTorqueKpKiFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_PID_TORQUE_KP_KI);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.kp >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.kp);
                tx[3] = static_cast<uint8_t>(0xFF & (frame.ki >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & frame.ki);
            }
            else if constexpr (std::is_same_v<T, SetPidFluxKpKiFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_PID_FLUX_KP_KI);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.kp >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.kp);
                tx[3] = static_cast<uint8_t>(0xFF & (frame.ki >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & frame.ki);
            }
            else if constexpr (std::is_same_v<T, SetPidSpeedKpKiFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_PID_SPEED_KP_KI);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.kp >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.kp);
                tx[3] = static_cast<uint8_t>(0xFF & (frame.ki >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & frame.ki);
            }
        },
        outgoing_frame);

    tx[5] = Crc8Autosar::calc(tx, FRAME_LEN - 1);

    spiTransfer(file_descriptors_[CHIP_SELECTS.at(motor)], tx, rx, FRAME_LEN,
                SPI_SPEED_HZ);

    // Frame integrity check
    const uint8_t rx_crc = Crc8Autosar::calc(rx, FRAME_LEN - 1);
    if (rx[5] != rx_crc)
    {
        LOG(WARNING) << "Received frame that failed integrity check. Expected CRC "
                     << static_cast<int>(rx_crc) << " but got " << static_cast<int>(rx[5])
                     << " for motor " << motor << "\n"
                     << "RX: " << static_cast<int>(rx[0]) << " "
                     << static_cast<int>(rx[1]) << " " << static_cast<int>(rx[2]) << " "
                     << static_cast<int>(rx[3]) << " " << static_cast<int>(rx[4]) << " "
                     << static_cast<int>(rx[5]);
        return;
    }

    switch (static_cast<StSpinResponseType>(rx[0]))
    {
        case StSpinResponseType::SPEED_AND_FAULTS:
        {
            motor_measured_speed_rpm_[motor] =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_faults_[motor] =
                static_cast<uint16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::IQ_AND_ID:
        {
            motor_iq_[motor] =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_id_[motor] =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
    }
}
