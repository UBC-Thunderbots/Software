#include "software/embedded/motor_controller/stspin_motor_controller.h"

#include <linux/spi/spidev.h>

#include <chrono>
#include <iomanip>
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

StSpinMotorController::StSpinMotorController(const RobotConstants& robot_constants)
    : robot_constants_(robot_constants),
      reset_gpio_(std::make_unique<GpioCharDev>(RESET_GPIO_PIN, GpioDirection::OUTPUT,
                                                GpioState::HIGH))
{
    for (const MotorIndex motor : driveMotors())
    {
        openSpiFileDescriptor(motor);
    }
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
        // sendAndReceiveFrame(motor, SetPidSpeedKpKiFrame{.kp = 0, .ki = 0});
        // sendAndReceiveFrame(motor, SetPidSpeedKpKiFrame{.kp = 300, .ki = 10});
        // sendAndReceiveFrame(motor, SetPidSpeedKpKiFrame{.kp = 500, .ki = 30});
        sendAndReceiveFrame(motor, SetPidSpeedKpKiFrame{.kp = 700, .ki = 30});
        sendAndReceiveFrame(motor, SetSpeedFeedForwardKaKvFrame{.ka = 0, .kv = 0});
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
    motor_faults.drive_enabled        = true;
    motor_faults.faults.clear();

    if (fault_flags == 0)
    {
        // No faults; early return
        return;
    }

    std::ostringstream oss;
    oss << "======= Faults For Motor " << motor << "=======\n";

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::DURATION))
    {
        oss << "DURATION: FOC rate too high\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::DURATION);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_VOLT))
    {
        oss << "OVER_VOLT: Over voltage\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_VOLT);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::UNDER_VOLT))
    {
        oss << "UNDER_VOLT: Under voltage\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::UNDER_VOLT);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_TEMP))
    {
        oss << "OVER_TEMP: Over temperature\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_TEMP);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::START_UP))
    {
        oss << "START_UP: Start up failed\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::START_UP);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SPEED_FDBK))
    {
        oss << "SPEED_FDBK: Speed feedback fault\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::SPEED_FDBK);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVER_CURR))
    {
        oss << "OVER_CURR: Over current\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVER_CURR);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SW_ERROR))
    {
        oss << "SW_ERROR: Software error\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::SW_ERROR);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::SAMPLE_FAULT))
    {
        oss << "SAMPLE_FAULT: Sample fault for testing purposes\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::SAMPLE_FAULT);
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::OVERCURR_SW))
    {
        oss << "OVERCURR_SW: Software over current\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::OVERCURR_SW);
        motor_faults.drive_enabled = false;
    }

    if (fault_flags & static_cast<uint16_t>(StSpinFaultCode::DP_FAULT))
    {
        oss << "DP_FAULT: Driver protection fault\n";
        motor_faults.faults.insert(TbotsProto::MotorFault::DP_FAULT);
        motor_faults.drive_enabled = false;
    }

    LOG(WARNING) << oss.str();
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

    // sendMotorStatusToPlotJuggler(motor);

    return motor_status_.at(motor).speed;
}

void StSpinMotorController::updateEuclideanVelocity(
    EuclideanSpace_t target_euclidean_velocity)
{
    const Vector local_velocity(target_euclidean_velocity[1],
                                -target_euclidean_velocity[0]);

    if (local_velocity.length() <= 0.01)
    {
        sendAndReceiveFrame(
            MotorIndex::FRONT_LEFT,
            SetSpeedFeedForwardKsFrame{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveFrame(
            MotorIndex::FRONT_RIGHT,
            SetSpeedFeedForwardKsFrame{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveFrame(
            MotorIndex::BACK_RIGHT,
            SetSpeedFeedForwardKsFrame{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveFrame(
            MotorIndex::BACK_LEFT,
            SetSpeedFeedForwardKsFrame{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        return;
    }

    const Angle direction = local_velocity.orientation();

    const Angle front_wheel_angle =
        Angle::fromDegrees(robot_constants_.front_wheel_angle_deg);
    const Angle back_wheel_angle =
        Angle::fromDegrees(robot_constants_.back_wheel_angle_deg);

    const int16_t front_left_ks = static_cast<int16_t>(
        MAX_SPEED_FEED_FORWARD_STATIC_GAIN *
        std::abs((direction - Angle::quarter() + front_wheel_angle).sin()));
    const int16_t front_right_ks = static_cast<int16_t>(
        MAX_SPEED_FEED_FORWARD_STATIC_GAIN *
        std::abs((direction + Angle::quarter() - front_wheel_angle).sin()));
    const int16_t back_right_ks = static_cast<int16_t>(
        MAX_SPEED_FEED_FORWARD_STATIC_GAIN *
        std::abs((direction + Angle::quarter() + back_wheel_angle).sin()));
    const int16_t back_left_ks = static_cast<int16_t>(
        MAX_SPEED_FEED_FORWARD_STATIC_GAIN *
        std::abs((direction - Angle::quarter() - back_wheel_angle).sin()));

    sendAndReceiveFrame(MotorIndex::FRONT_LEFT,
                        SetSpeedFeedForwardKsFrame{.ks = front_left_ks});
    sendAndReceiveFrame(MotorIndex::FRONT_RIGHT,
                        SetSpeedFeedForwardKsFrame{.ks = front_right_ks});
    sendAndReceiveFrame(MotorIndex::BACK_RIGHT,
                        SetSpeedFeedForwardKsFrame{.ks = back_right_ks});
    sendAndReceiveFrame(MotorIndex::BACK_LEFT,
                        SetSpeedFeedForwardKsFrame{.ks = back_left_ks});
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
        // Log RX buffer and expected vs. actual CRC
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0') << "Expected CRC 0x"
            << std::setw(2) << static_cast<int>(rx[5]) << " but got 0x" << std::setw(2)
            << static_cast<int>(rx_crc) << ". RX: ";
        for (size_t i = 0; i < FRAME_LEN; ++i)
        {
            oss << "0x" << std::setw(2) << static_cast<int>(rx[i]) << " ";
        }

        LOG(WARNING) << "Frame #" << motor_status_[motor].frame_count
                     << " received from motor " << motor << " failed integrity check. "
                     << oss.str();
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
            else if constexpr (std::is_same_v<T, SetSpeedFeedForwardKaKvFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_SPEED_FEED_FORWARD_KA_KV);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.ka >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.ka);
                tx[3] = static_cast<uint8_t>(0xFF & (frame.kv >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & frame.kv);
            }
            else if constexpr (std::is_same_v<T, SetSpeedFeedForwardKsFrame>)
            {
                tx[0] = static_cast<uint8_t>(StSpinOpcode::SET_SPEED_FEED_FORWARD_KS);
                tx[1] = static_cast<uint8_t>(0xFF & (frame.ks >> 8));
                tx[2] = static_cast<uint8_t>(0xFF & frame.ks);
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
