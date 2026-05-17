#include "software/embedded/services/motor.h"

#include "proto/tbots_software_msgs.pb.h"
#include "software/embedded/motor_controller/motor_board.h"
#include "software/embedded/motor_controller/stspin_motor_controller.h"
#include "software/embedded/motor_controller/tmc_motor_controller.h"
#include "software/logger/logger.h"

MotorService::MotorService(const robot_constants::RobotConstants& robot_constants)
    : motor_controller_(setupMotorController()),
      robot_constants_(robot_constants),
      euclidean_to_four_wheel_(robot_constants),
      drive_motor_mps_per_rpm_(2 * M_PI * robot_constants.wheel_radius_meters / 60)
{
}

void MotorService::setup()
{
    prev_wheel_velocities_   = WheelSpace_t::Zero();
    target_wheel_velocities_ = WheelSpace_t::Zero();
    dribbler_target_rpm_     = 0;

    motor_controller_->setup();
}

void MotorService::reset()
{
    motor_controller_->reset();
}

std::unique_ptr<MotorController> MotorService::setupMotorController()
{
    if constexpr (MOTOR_BOARD == MotorBoard::TRINAMIC)
    {
        return std::make_unique<TmcMotorController>();
    }
    else
    {
        return std::make_unique<StSpinMotorController>();
    }
}

TbotsProto::MotorStatus MotorService::createMotorStatus(
    const WheelSpace_t& current_wheel_velocities, const double dribbler_rpm) const
{
    TbotsProto::MotorStatus motor_status;

    for (const MotorIndex motor : reflective_enum::values<MotorIndex>())
    {
        const MotorFaultIndicator& motor_faults = motor_controller_->checkFaults(motor);

        if (motor != MotorIndex::DRIBBLER)
        {
            TbotsProto::DriveUnit drive_status;
            drive_status.set_enabled(motor_faults.drive_enabled);

            for (const TbotsProto::MotorFault& fault : motor_faults.faults)
            {
                drive_status.add_motor_faults(fault);
            }

            if (motor == MotorIndex::FRONT_LEFT)
            {
                *(motor_status.mutable_front_left()) = drive_status;
            }
            if (motor == MotorIndex::FRONT_RIGHT)
            {
                *(motor_status.mutable_front_right()) = drive_status;
            }
            if (motor == MotorIndex::BACK_LEFT)
            {
                *(motor_status.mutable_back_left()) = drive_status;
            }
            if (motor == MotorIndex::BACK_RIGHT)
            {
                *(motor_status.mutable_back_right()) = drive_status;
            }
        }
        else
        {
            TbotsProto::DribblerStatus dribbler_status;
            dribbler_status.set_dribbler_rpm(static_cast<float>(dribbler_rpm));
            dribbler_status.set_enabled(motor_faults.drive_enabled);

            for (const TbotsProto::MotorFault& fault : motor_faults.faults)
            {
                dribbler_status.add_motor_faults(fault);
            }

            *(motor_status.mutable_dribbler()) = dribbler_status;
        }
    }

    motor_status.mutable_front_left()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_front_right()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_back_left()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_back_right()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX]));

    return motor_status;
}

TbotsProto::MotorStatus MotorService::poll(const TbotsProto::MotorControl& motor_control,
                                           const double time_elapsed_since_last_poll_s)
{
    if (anyMotorRequiresReset())
    {
        LOG(INFO) << "Resetting motors due to fault indicators requiring reset";
        trackMotorReset();
        setup();
    }

    const Eigen::Vector4<int> target_wheel_rpms =
        (target_wheel_velocities_ / drive_motor_mps_per_rpm_).cast<int>();

    const Eigen::Vector4<int> current_wheel_rpms = {
        motor_controller_->readThenWriteVelocity(
            MotorIndex::FRONT_RIGHT, target_wheel_rpms[FRONT_RIGHT_WHEEL_SPACE_INDEX]),
        motor_controller_->readThenWriteVelocity(
            MotorIndex::FRONT_LEFT, target_wheel_rpms[FRONT_LEFT_WHEEL_SPACE_INDEX]),
        motor_controller_->readThenWriteVelocity(
            MotorIndex::BACK_LEFT, target_wheel_rpms[BACK_LEFT_WHEEL_SPACE_INDEX]),
        motor_controller_->readThenWriteVelocity(
            MotorIndex::BACK_RIGHT, target_wheel_rpms[BACK_RIGHT_WHEEL_SPACE_INDEX]),
    };

    const double dribbler_rpm = motor_controller_->readThenWriteVelocity(
        MotorIndex::DRIBBLER, dribbler_target_rpm_);

    const WheelSpace_t current_wheel_velocities =
        current_wheel_rpms.cast<double>() * drive_motor_mps_per_rpm_;

    TbotsProto::MotorStatus motor_status =
        createMotorStatus(current_wheel_velocities, dribbler_rpm);

    if (std::abs(current_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] -
                 prev_wheel_velocities_[FRONT_RIGHT_WHEEL_SPACE_INDEX]) >
        RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        motor_controller_->immediatelyDisable();
        LOG(FATAL) << "Front right motor runaway";
    }
    else if (std::abs(current_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[FRONT_LEFT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        motor_controller_->immediatelyDisable();
        LOG(FATAL) << "Front left motor runaway";
    }
    else if (std::abs(current_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[BACK_LEFT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        motor_controller_->immediatelyDisable();
        LOG(FATAL) << "Back left motor runaway";
    }
    else if (std::abs(current_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] -
                      prev_wheel_velocities_[BACK_RIGHT_WHEEL_SPACE_INDEX]) >
             RUNAWAY_PROTECTION_THRESHOLD_MPS)
    {
        motor_controller_->immediatelyDisable();
        LOG(FATAL) << "Back right motor runaway";
    }

    // Convert to Euclidean velocity_delta
    const EuclideanSpace_t current_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(current_wheel_velocities);

    motor_status.mutable_local_velocity()->set_x_component_meters(
        current_euclidean_velocity[1]);
    motor_status.mutable_local_velocity()->set_y_component_meters(
        -current_euclidean_velocity[0]);
    motor_status.mutable_angular_velocity()->set_radians_per_second(
        current_euclidean_velocity[2]);

    // Get target wheel velocities from the primitive
    if (motor_control.has_direct_per_wheel_control())
    {
        const auto& direct_per_wheel = motor_control.direct_per_wheel_control();

        target_wheel_velocities_ = {
            direct_per_wheel.front_right_wheel_velocity(),
            direct_per_wheel.front_left_wheel_velocity(),
            direct_per_wheel.back_left_wheel_velocity(),
            direct_per_wheel.back_right_wheel_velocity(),
        };
    }
    else if (motor_control.has_direct_velocity_control())
    {
        const auto& direct_velocity = motor_control.direct_velocity_control();

        const EuclideanSpace_t target_euclidean_velocity = {
            -direct_velocity.velocity().y_component_meters(),
            direct_velocity.velocity().x_component_meters(),
            direct_velocity.angular_velocity().radians_per_second()};

        target_wheel_velocities_ =
            euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity);
    }

    // Ramp the target velocities to keep acceleration compared to current velocities
    // within safe bounds
    target_wheel_velocities_ = euclidean_to_four_wheel_.rampWheelVelocity(
        prev_wheel_velocities_, target_wheel_velocities_, time_elapsed_since_last_poll_s);

    prev_wheel_velocities_ = target_wheel_velocities_;

    // Get target dribbler rpm from the primitive
    int target_dribbler_rpm;
    if (motor_control.drive_control_case() ==
        TbotsProto::MotorControl::DriveControlCase::DRIVE_CONTROL_NOT_SET)
    {
        target_dribbler_rpm = 0;
    }
    else
    {
        target_dribbler_rpm = motor_control.dribbler_speed_rpm();
    }

    // Ramp the dribbler velocity
    // Clamp the max acceleration
    int max_dribbler_delta_rpm = static_cast<int>(
        DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 * time_elapsed_since_last_poll_s);
    int delta_rpm = std::clamp(target_dribbler_rpm - dribbler_target_rpm_,
                               -max_dribbler_delta_rpm, max_dribbler_delta_rpm);
    dribbler_target_rpm_ += delta_rpm;

    // Clamp to the max rpm
    int max_dribbler_rpm = std::abs(robot_constants_.max_force_dribbler_speed_rpm);
    dribbler_target_rpm_ =
        std::clamp(dribbler_target_rpm_, -max_dribbler_rpm, max_dribbler_rpm);

    motor_status.mutable_dribbler()->set_dribbler_rpm(
        static_cast<float>(dribbler_target_rpm_));

    return motor_status;
}

void MotorService::trackMotorReset()
{
    const auto now = std::chrono::system_clock::now();

    if (num_tracked_motor_resets_ == 0)
    {
        tracked_motor_reset_start_time_ = now;
        num_tracked_motor_resets_       = 1;
        return;
    }

    const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
                               now - tracked_motor_reset_start_time_)
                               .count();

    if (elapsed_s < MOTOR_RESET_TIME_THRESHOLD_S)
    {
        num_tracked_motor_resets_++;
    }
    else
    {
        tracked_motor_reset_start_time_ = now;
        num_tracked_motor_resets_       = 1;
    }

    if (num_tracked_motor_resets_ > MOTOR_RESET_THRESHOLD_COUNT)
    {
        LOG(FATAL) << "Motors reset too frequently (" << num_tracked_motor_resets_
                   << " times in " << elapsed_s
                   << " seconds). Thunderloop crashing for safety.";
    }
}

bool MotorService::anyMotorRequiresReset() const
{
    return std::any_of(reflective_enum::values<MotorIndex>().begin(),
                       reflective_enum::values<MotorIndex>().end(),
                       [&](const MotorIndex motor)
                       { return motor_controller_->checkFaults(motor).requiresReset(); });
}
