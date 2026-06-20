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

StSpinMotorController::StSpinMotorController(
    const robot_constants::RobotConstants& robot_constants)
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
        sendAndReceiveMessage(motor,
                              SetPidSpeedKpKiMessage{.kp = SPEED_PID_PROPORTIONAL_GAIN,
                                                     .ki = SPEED_PID_INTEGRAL_GAIN});
        sendAndReceiveMessage(motor,
                              SetPidTorqueKpKiMessage{.kp = TORQUE_PID_PROPORTIONAL_GAIN,
                                                      .ki = TORQUE_PID_INTEGRAL_GAIN});
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
        // No change in faults
        return;
    }

    motor_status.fault_flags          = fault_flags;
    MotorFaultIndicator& motor_faults = motor_status.faults;
    motor_faults.drive_enabled        = true;
    motor_faults.faults.clear();

    if (fault_flags == 0)
    {
        // No faults
        return;
    }

    // TODO #3748 Use a helper, stop regenerating the stream object.
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

    const auto outgoing_message = SetTargetSpeedMessage{
        .motor_enabled          = motor_status_.at(motor).enabled,
        .motor_target_speed_rpm = static_cast<int16_t>(target_velocity),
    };

    sendAndReceiveMessage(motor, outgoing_message);

    return motor_status_.at(motor).speed;
}

void StSpinMotorController::updateEuclideanVelocity(
    EuclideanSpace_t target_euclidean_velocity)
{
#if 0
    const Vector local_velocity(target_euclidean_velocity[1],
                                target_euclidean_velocity[0]);

    if (local_velocity.length() <= MINIMUM_SPEED_FOR_FEED_FORWARD)
    {
        sendAndReceiveMessage(
            MotorIndex::FRONT_LEFT,
            SetSpeedFeedForwardKsMessage{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveMessage(
            MotorIndex::FRONT_RIGHT,
            SetSpeedFeedForwardKsMessage{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveMessage(
            MotorIndex::BACK_RIGHT,
            SetSpeedFeedForwardKsMessage{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
        sendAndReceiveMessage(
            MotorIndex::BACK_LEFT,
            SetSpeedFeedForwardKsMessage{.ks = MIN_SPEED_FEED_FORWARD_STATIC_GAIN});
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

    sendAndReceiveMessage(MotorIndex::FRONT_LEFT,
                          SetSpeedFeedForwardKsMessage{.ks = front_left_ks});
    sendAndReceiveMessage(MotorIndex::FRONT_RIGHT,
                          SetSpeedFeedForwardKsMessage{.ks = front_right_ks});
    sendAndReceiveMessage(MotorIndex::BACK_RIGHT,
                          SetSpeedFeedForwardKsMessage{.ks = back_right_ks});
    sendAndReceiveMessage(MotorIndex::BACK_LEFT,
                          SetSpeedFeedForwardKsMessage{.ks = back_left_ks});
#endif
    sendMotorStatusToPlotJuggler(MotorIndex::FRONT_LEFT);
    sendMotorStatusToPlotJuggler(MotorIndex::FRONT_RIGHT);
    sendMotorStatusToPlotJuggler(MotorIndex::BACK_LEFT);
    sendMotorStatusToPlotJuggler(MotorIndex::BACK_RIGHT);
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

void StSpinMotorController::sendAndReceiveMessage(const MotorIndex motor,
                                                  const OutgoingMessage& outgoing_message)
{
    std::array<uint8_t, MESSAGE_SIZE> tx{};
    std::array<uint8_t, MESSAGE_SIZE> rx{};

    motor_status_[motor].seq_num++;

    populateTx(motor, outgoing_message, tx);

    std::vector<uint8_t> received_data;
    for (unsigned int attempt = 0; attempt < MAX_RESYNC_ATTEMPTS; ++attempt)
    {
        spiTransfer(spi_fds_[motor], tx.data(), rx.data(), MESSAGE_SIZE, SPI_SPEED_HZ);
        received_data.insert(received_data.end(), rx.begin(), rx.end());

        auto delimiter_pos =
            std::find(received_data.begin(), received_data.end(), MESSAGE_DELIMITER);

        if (delimiter_pos == received_data.end())
        {
            // No delimiter byte found, discard everything
            received_data.clear();
            continue;
        }

        if (std::distance(delimiter_pos, received_data.end()) < MESSAGE_SIZE)
        {
            // Not enough bytes after delimiter byte for a full message,
            // wait for more bytes
            continue;
        }

        // Message integrity check
        const uint8_t expected_crc = *std::next(delimiter_pos, MESSAGE_SIZE - 1);
        const uint8_t actual_crc = Crc8Autosar::calc(&(*delimiter_pos), MESSAGE_SIZE - 1);
        if (expected_crc == actual_crc)
        {
            // Only the low byte of seq_num is transmitted in tx[1], so compare against
            // the low byte here too. Comparing the full uint16_t would never match once
            // seq_num wraps past 255 and would spin the loop forever.
            const uint8_t ack_seq_num = *std::next(delimiter_pos);
            if (ack_seq_num == static_cast<uint8_t>(motor_status_.at(motor).seq_num))
            {
                // The message we just sent was acknowledged, we can stop resending
                // and waiting for a response. Plot the time waited since the previous
                // completed frame so we can spot SPI stalls / resync delays.
                MotorStatus& motor_status = motor_status_.at(motor);
                const auto now            = std::chrono::steady_clock::now();
                if (motor_status.last_frame_complete_time !=
                    std::chrono::steady_clock::time_point{})
                {
                    const double frame_wait_ms =
                        std::chrono::duration<double, std::milli>(
                            now - motor_status.last_frame_complete_time)
                            .count();
                    LOG(PLOTJUGGLER) << *createPlotJugglerValue({
                        {"frame_wait_ms_" + motor, frame_wait_ms},
                    });
                }
                motor_status.last_frame_complete_time = now;

                std::array<uint8_t, MESSAGE_SIZE> message{};
                std::copy(delimiter_pos, std::next(delimiter_pos, MESSAGE_SIZE),
                          message.begin());
                processRx(motor, message);
                return;
            }
        }

        received_data.erase(received_data.begin(), std::next(delimiter_pos));
    }

    // Resync did not complete within the attempt budget. Degrade gracefully by giving
    // up on this message rather than blocking the caller (e.g. setup() during init).
    LOG(WARNING) << "Motor " << motor << " did not acknowledge message (seq_num "
                 << motor_status_.at(motor).seq_num << ") after " << MAX_RESYNC_ATTEMPTS
                 << " SPI attempts; giving up";
}

void StSpinMotorController::populateTx(const MotorIndex motor,
                                       const OutgoingMessage& outgoing_message,
                                       std::array<uint8_t, MESSAGE_SIZE>& tx)
{
    tx[0] = MESSAGE_DELIMITER;
    tx[1] = motor_status_.at(motor).seq_num;

    std::visit(
        [&]<typename TMessage>(TMessage&& message)
        {
            using T = std::decay_t<TMessage>;
            if constexpr (std::is_same_v<T, NoOpMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::NO_OP);
            }
            else if constexpr (std::is_same_v<T, SetTargetSpeedMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_TARGET_SPEED);
                tx[3] = static_cast<uint8_t>(message.motor_enabled);
                tx[4] =
                    static_cast<uint8_t>(0xFF & (message.motor_target_speed_rpm >> 8));
                tx[5] = static_cast<uint8_t>(0xFF & message.motor_target_speed_rpm);
            }
            else if constexpr (std::is_same_v<T, SetTargetTorqueMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_TARGET_TORQUE);
                tx[3] = static_cast<uint8_t>(message.motor_enabled);
                tx[4] = static_cast<uint8_t>(0xFF & (message.motor_target_torque >> 8));
                tx[5] = static_cast<uint8_t>(0xFF & message.motor_target_torque);
            }
            else if constexpr (std::is_same_v<T, SetResponseTypeMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_RESPONSE_TYPE);
                tx[3] = static_cast<uint8_t>(message.response_type);
            }
            else if constexpr (std::is_same_v<T, SetPidTorqueKpKiMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_PID_TORQUE_KP_KI);
                tx[3] = static_cast<uint8_t>(0xFF & (message.kp >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & message.kp);
                tx[5] = static_cast<uint8_t>(0xFF & (message.ki >> 8));
                tx[6] = static_cast<uint8_t>(0xFF & message.ki);
            }
            else if constexpr (std::is_same_v<T, SetPidFluxKpKiMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_PID_FLUX_KP_KI);
                tx[3] = static_cast<uint8_t>(0xFF & (message.kp >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & message.kp);
                tx[5] = static_cast<uint8_t>(0xFF & (message.ki >> 8));
                tx[6] = static_cast<uint8_t>(0xFF & message.ki);
            }
            else if constexpr (std::is_same_v<T, SetPidSpeedKpKiMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_PID_SPEED_KP_KI);
                tx[3] = static_cast<uint8_t>(0xFF & (message.kp >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & message.kp);
                tx[5] = static_cast<uint8_t>(0xFF & (message.ki >> 8));
                tx[6] = static_cast<uint8_t>(0xFF & message.ki);
            }
            else if constexpr (std::is_same_v<T, SetSpeedFeedForwardKaKvMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_SPEED_FEED_FORWARD_KA_KV);
                tx[3] = static_cast<uint8_t>(0xFF & (message.ka >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & message.ka);
                tx[5] = static_cast<uint8_t>(0xFF & (message.kv >> 8));
                tx[6] = static_cast<uint8_t>(0xFF & message.kv);
            }
            else if constexpr (std::is_same_v<T, SetSpeedFeedForwardKsMessage>)
            {
                tx[2] = static_cast<uint8_t>(StSpinOpcode::SET_SPEED_FEED_FORWARD_KS);
                tx[3] = static_cast<uint8_t>(0xFF & (message.ks >> 8));
                tx[4] = static_cast<uint8_t>(0xFF & message.ks);
            }
        },
        outgoing_message);

    tx[7] = Crc8Autosar::calc(tx.data(), MESSAGE_SIZE - 1);
}

void StSpinMotorController::processRx(const MotorIndex motor,
                                      const std::array<uint8_t, MESSAGE_SIZE>& rx)
{
    switch (static_cast<StSpinResponseType>(rx[2]))
    {
        case StSpinResponseType::SPEED_AND_FAULTS:
        {
            motor_status_[motor].speed =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            const uint16_t fault_flags =
                static_cast<uint16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            updateFaults(motor, fault_flags);
            break;
        }
        case StSpinResponseType::IQ_AND_ID:
        {
            motor_status_[motor].iq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].id =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            break;
        }
        case StSpinResponseType::VQ_AND_VD:
        {
            motor_status_[motor].vq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].vd =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            break;
        }
        case StSpinResponseType::PHASE_CURRENT_AND_VOLTAGE:
        {
            motor_status_[motor].phase_current =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].phase_voltage =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            break;
        }
        case StSpinResponseType::IQ_AND_IQ_REF:
        {
            motor_status_[motor].iq =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].iq_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            break;
        }
        case StSpinResponseType::ID_AND_ID_REF:
        {
            motor_status_[motor].id =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].id_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
            break;
        }
        case StSpinResponseType::SPEED_AND_SPEED_REF:
        {
            motor_status_[motor].speed =
                static_cast<int16_t>((static_cast<uint16_t>(rx[3]) << 8) | rx[4]);
            motor_status_[motor].speed_ref =
                static_cast<int16_t>((static_cast<uint16_t>(rx[5]) << 8) | rx[6]);
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
        sendAndReceiveMessage(motor, SetResponseTypeMessage{response_type});
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
