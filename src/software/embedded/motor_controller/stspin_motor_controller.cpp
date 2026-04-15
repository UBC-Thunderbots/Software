#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <linux/spi/spidev.h>

#include <chrono>
#include <thread>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include "cppcrc.h"
#pragma GCC diagnostic pop

#include "proto/message_translation/tbots_protobuf.h"
#include "software/embedded/gpio/gpio_char_dev.h"
#include "software/embedded/motor_controller/stspin_types.h"
#include "software/embedded/spi_utils.h"
#include "software/logger/logger.h"

// AUTOSAR variant of CRC-8
// (https://reveng.sourceforge.io/crc-catalogue/all.htm#crc.cat.crc-8-autosar)
using Crc8Autosar = crc_utils::crc<uint8_t, 0x2F, 0xFF, false, false, 0xFF>;

StSpinMotorController::StSpinMotorController()
    : reset_gpio_(std::make_unique<GpioCharDev>(RESET_GPIO_PIN, GpioDirection::OUTPUT,
                                                GpioState::HIGH))
{
    for (const MotorIndex motor : driveMotors())
    {
        openSpiFileDescriptor(motor);
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

    for (const MotorIndex motor : reflective_enum::values<MotorIndex>())
    {
        motor_status_[motor]         = MotorStatus();
        motor_status_[motor].enabled = true;
    }

    for (const MotorIndex motor : driveMotors())
    {
        sendAndReceiveFrame(motor, SetPidSpeedKpKiFrame{.kp = 806, .ki = 154});
    }
}

void StSpinMotorController::reset()
{
    reset_gpio_->setValue(GpioState::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    reset_gpio_->setValue(GpioState::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

const MotorFaultIndicator& StSpinMotorController::checkFaults(const MotorIndex motor)
{
    return motor_status_.at(motor).faults;
}

void StSpinMotorController::updateFaults(const MotorIndex motor,
                                         const uint16_t fault_flags)
{
    MotorStatus& motor_status = motor_status_.at(motor);

    if (motor_status.fault_flags == fault_flags)
    {
        // No change in faults; early return
        return;
    }

    motor_status.fault_flags          = fault_flags;
    MotorFaultIndicator& motor_faults = motor_status.faults;
    motor_faults.faults.clear();

    LOG(WARNING) << "======= Faults For Motor " << motor << "=======";

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::DURATION))
    {
        LOG(WARNING) << "DURATION: FOC rate too high";
        motor_faults.faults.insert(TbotsProto::MotorFault::DURATION);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_VOLT))
    {
        LOG(WARNING) << "OVER_VOLT: Over voltage";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_VOLT);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::UNDER_VOLT))
    {
        LOG(WARNING) << "UNDER_VOLT: Under voltage";
        motor_faults.faults.insert(TbotsProto::MotorFault::UNDER_VOLT);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_TEMP))
    {
        LOG(WARNING) << "OVER_TEMP: Over temperature";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_TEMP);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::START_UP))
    {
        LOG(WARNING) << "START_UP: Start up failed";
        motor_faults.faults.insert(TbotsProto::MotorFault::START_UP);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SPEED_FDBK))
    {
        LOG(WARNING) << "SPEED_FDBK: Speed feedback fault";
        motor_faults.faults.insert(TbotsProto::MotorFault::SPEED_FDBK);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_CURR))
    {
        LOG(WARNING) << "OVER_CURR: Over current";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_CURR);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SW_ERROR))
    {
        LOG(WARNING) << "SW_ERROR: Software error";
        motor_faults.faults.insert(TbotsProto::MotorFault::SW_ERROR);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SAMPLE_FAULT))
    {
        LOG(WARNING) << "SAMPLE_FAULT: Sample fault for testing purposes";
        motor_faults.faults.insert(TbotsProto::MotorFault::SAMPLE_FAULT);
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVERCURR_SW))
    {
        LOG(INFO) << "OVERCURR_SW: Software over current";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVERCURR_SW);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::DP_FAULT))
    {
        LOG(WARNING) << "DP_FAULT: Driver protection fault";
        motor_faults.faults.insert(TbotsProto::MotorFault::DP_FAULT);
        motor_faults.drive_enabled = false;
    }
}

int StSpinMotorController::readThenWriteVelocity(const MotorIndex motor,
                                                 const int target_velocity)
{
    if (motor == MotorIndex::DRIBBLER)
    {
        return 0;
    }

    const auto outgoing_frame = SetTargetSpeedFrame{
        .motor_enabled          = motor_status_.at(motor).enabled,
        .motor_target_speed_rpm = static_cast<int16_t>(target_velocity),
    };

    sendAndReceiveFrame(motor, outgoing_frame);

    sendMotorStatusToPlotJuggler(motor);

    return motor_status_.at(motor).speed;
}

void StSpinMotorController::immediatelyDisable()
{
    for (const MotorIndex motor : reflective_enum::values<MotorIndex>())
    {
        motor_status_[motor].enabled = false;
        readThenWriteVelocity(motor, 0);
    }
}

void StSpinMotorController::openSpiFileDescriptor(const MotorIndex motor)
{
    spi_fds_[motor] = open(SPI_PATHS.at(motor), O_RDWR);
    CHECK(spi_fds_[motor] >= 0)
        << "can't open device: " << motor << "error: " << strerror(errno);

    int ret = ioctl(spi_fds_[motor], SPI_IOC_WR_MODE32, &SPI_MODE);
    CHECK(ret != -1) << "can't set spi mode for: " << motor
                     << "error: " << strerror(errno);

    ret = ioctl(spi_fds_[motor], SPI_IOC_WR_BITS_PER_WORD, &SPI_BITS);
    CHECK(ret != -1) << "can't set bits_per_word for: " << motor
                     << "error: " << strerror(errno);

    ret = ioctl(spi_fds_[motor], SPI_IOC_WR_MAX_SPEED_HZ, &MAX_SPI_SPEED_HZ);
    CHECK(ret != -1) << "can't set spi max speed hz for: " << motor
                     << "error: " << strerror(errno);
}

void StSpinMotorController::sendAndReceiveFrame(const MotorIndex motor,
                                                const OutgoingFrame& outgoing_frame)
{
    std::array<uint8_t, FRAME_LEN> tx{};
    std::array<uint8_t, FRAME_LEN> rx{};

    populateTx(outgoing_frame, tx);

    spiTransfer(spi_fds_[motor], tx.data(), rx.data(), FRAME_LEN, SPI_SPEED_HZ);

    motor_status_[motor].frame_count++;

    // Frame integrity check
    const uint8_t rx_crc = Crc8Autosar::calc(rx.data(), FRAME_LEN - 1);
    if (rx[5] != rx_crc)
    {
        LOG(WARNING) << "Frame #" << motor_status_[motor].frame_count
                     << " received from motor " << motor
                     << " failed integrity check. Expected CRC "
                     << static_cast<int>(rx_crc) << " but got " << static_cast<int>(rx[5])
                     << ". RX: " << static_cast<int>(rx[0]) << " "
                     << static_cast<int>(rx[1]) << " " << static_cast<int>(rx[2]) << " "
                     << static_cast<int>(rx[3]) << " " << static_cast<int>(rx[4]) << " "
                     << static_cast<int>(rx[5]);
        return;
    }

    processRx(motor, rx);
}

void StSpinMotorController::populateTx(const OutgoingFrame& outgoing_frame,
                                       std::array<uint8_t, FRAME_LEN>& tx)
{
    std::visit(
        [&]<typename TFrame>(TFrame&& frame)
        {
            using T = std::decay_t<TFrame>;
            if constexpr (std::is_same_v<T, NoOpFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::NO_OP);
            }
            else if constexpr (std::is_same_v<T, SetTargetSpeedFrame>)
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
            else if constexpr (std::is_same_v<T, SetPidSpeedKdFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_PID_SPEED_KD);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.kd >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.kd);
            }
        },
        outgoing_frame);

    tx[5] = Crc8Autosar::calc(tx.data(), FRAME_LEN - 1);
}

void StSpinMotorController::processRx(const MotorIndex motor,
                                      const std::array<uint8_t, FRAME_LEN>& rx)
{
    switch (static_cast<StSpinResponseType>(rx[0]))
    {
        case StSpinResponseType::SPEED_AND_FAULTS:
        {
            motor_status_[motor].speed =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            const uint16_t fault_flags =
                static_cast<uint16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            updateFaults(motor, fault_flags);
            break;
        }
        case StSpinResponseType::IQ_AND_ID:
        {
            motor_status_[motor].iq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].id =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::VQ_AND_VD:
        {
            motor_status_[motor].vq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].vd =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::PHASE_CURRENT_AND_VOLTAGE:
        {
            motor_status_[motor].phase_current =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].phase_voltage =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::IQ_AND_IQ_REF:
        {
            motor_status_[motor].iq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].iq_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::ID_AND_ID_REF:
        {
            motor_status_[motor].id =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].id_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
        case StSpinResponseType::SPEED_AND_SPEED_REF:
        {
            motor_status_[motor].speed =
                static_cast<int16_t>((static_cast<uint16_t>(rx[1]) << 8) | rx[2]);
            motor_status_[motor].speed_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            break;
        }
    }
}

void StSpinMotorController::sendMotorStatusToPlotJuggler(const MotorIndex motor)
{
    for (const StSpinResponseType response_type :
         {StSpinResponseType::SPEED_AND_SPEED_REF, StSpinResponseType::IQ_AND_IQ_REF,
          StSpinResponseType::ID_AND_ID_REF, StSpinResponseType::SPEED_AND_FAULTS})
    {
        sendAndReceiveFrame(motor, SetResponseTypeFrame{response_type});
    }

    const MotorStatus& motor_status = motor_status_.at(motor);

    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
        {"speed_" + motor, motor_status.speed},
        {"speed_ref_" + motor, motor_status.speed_ref},
        {"iq_" + motor, motor_status.iq},
        {"iq_ref_" + motor, motor_status.iq_ref},
        {"id_" + motor, motor_status.id},
        {"id_ref_" + motor, motor_status.id_ref},
    });
}
