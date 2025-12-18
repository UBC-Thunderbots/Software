#include "software/embedded/services/motor.h"

#include "proto/tbots_software_msgs.pb.h"
#include "software/embedded/motor_controller/motor_board.h"
#include "software/embedded/motor_controller/stspin_motor_controller.h"
#include "software/embedded/motor_controller/tmc_motor_controller.h"
#include "software/logger/logger.h"

static const uint32_t NUM_RETRIES_SPI = 3;

static double RUNAWAY_PROTECTION_THRESHOLD_MPS         = 2.00;
static int DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 = 10000;


MotorService::MotorService(const RobotConstants_t& robot_constants,
                           int control_loop_frequency_hz)
    : motor_controller_(setupMotorController()),
      robot_constants_(robot_constants),
      euclidean_to_four_wheel_(robot_constants),
      dribbler_ramp_rpm_(0),
      tracked_motor_fault_start_time_(std::nullopt)
{
}

MotorService::~MotorService() {}

void MotorService::setup()
{
    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        cached_motor_faults_[motor] = MotorFaultIndicator();
    }

    const auto now                             = std::chrono::system_clock::now();
    long int total_duration_since_last_fault_s = 0;
    if (tracked_motor_fault_start_time_.has_value())
    {
        total_duration_since_last_fault_s =
            std::chrono::duration_cast<std::chrono::seconds>(
                now - tracked_motor_fault_start_time_.value())
                .count();
    }

    if (tracked_motor_fault_start_time_.has_value() &&
        total_duration_since_last_fault_s < MOTOR_FAULT_TIME_THRESHOLD_S)
    {
        num_tracked_motor_resets_++;
    }
    else
    {
        tracked_motor_fault_start_time_ = std::make_optional(now);
        num_tracked_motor_resets_       = 1;
    }

    if (tracked_motor_fault_start_time_.has_value() &&
        num_tracked_motor_resets_ > MOTOR_FAULT_THRESHOLD_COUNT)
    {
        LOG(FATAL) << "In the last " << total_duration_since_last_fault_s
                   << "s, the motor board has reset " << num_tracked_motor_resets_
                   << " times. Thunderloop crashing for safety.";
    }

    prev_wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};

    motor_controller_->setup();

    is_initialized_ = true;
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

TbotsProto::MotorStatus MotorService::updateMotorStatus(double front_left_velocity_mps,
                                                        double front_right_velocity_mps,
                                                        double back_left_velocity_mps,
                                                        double back_right_velocity_mps,
                                                        double dribbler_rpm)
{
    TbotsProto::MotorStatus motor_status;

    cached_motor_faults_[motor_fault_detector_] =
        motor_controller_->checkDriverFault(motor_fault_detector_);

    for (const MotorIndex& motor : reflective_enum::values<MotorIndex>())
    {
        if (motor != MotorIndex::DRIBBLER)
        {
            TbotsProto::DriveUnit drive_status;
            drive_status.set_enabled(cached_motor_faults_[motor].drive_enabled);

            for (const TbotsProto::MotorFault& fault :
                 cached_motor_faults_[motor].motor_faults)
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
            dribbler_status.set_enabled(cached_motor_faults_[motor].drive_enabled);
            for (const TbotsProto::MotorFault& fault :
                 cached_motor_faults_[motor].motor_faults)
            {
                dribbler_status.add_motor_faults(fault);
            }

            *(motor_status.mutable_dribbler()) = dribbler_status;
        }
    }

    motor_status.mutable_front_left()->set_wheel_velocity(
        static_cast<float>(front_left_velocity_mps));
    motor_status.mutable_front_right()->set_wheel_velocity(
        static_cast<float>(front_right_velocity_mps));
    motor_status.mutable_back_left()->set_wheel_velocity(
        static_cast<float>(back_left_velocity_mps));
    motor_status.mutable_back_right()->set_wheel_velocity(
        static_cast<float>(back_right_velocity_mps));

    motor_fault_detector_ =
        static_cast<MotorIndex>((static_cast<int>(motor_fault_detector_) + 1) %
                                reflective_enum::size<MotorIndex>());

    return motor_status;
}

TbotsProto::MotorStatus MotorService::poll(const TbotsProto::MotorControl& motor,
                                           double time_elapsed_since_last_poll_s)
{
    if (motor_controller_->earlyPoll() != MotorControllerStatus::OK)
    {
        is_initialized_ = false;
    }

    // checks if any motor has reset, sends a log message if so
    for (const MotorIndex& motor_index : reflective_enum::values<MotorIndex>())
    {
        if (requiresMotorReinit(motor_index))
        {
            LOG(DEBUG) << "RESET DETECTED FOR MOTOR: " << motor_index;
            is_initialized_ = false;
        }
    }

    if (!is_initialized_)
    {
        LOG(INFO) << "MotorService re-initializing";
        setup();
    }

    // Get current wheel electical RPM (don't account for pole pairs). We will use these
    // for robot status feedback We assume the motors have ramped to the expected RPM from
    // the previous iteration.
    double front_right_velocity = motor_controller_->readThenWriteVelocity(
                                      MotorIndex::FRONT_RIGHT, front_right_target_rpm) *
                                  MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double front_left_velocity = motor_controller_->readThenWriteVelocity(
                                     MotorIndex::FRONT_LEFT, front_left_target_rpm) *
                                 MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double back_right_velocity = motor_controller_->readThenWriteVelocity(
                                     MotorIndex::BACK_RIGHT, back_right_target_rpm) *
                                 MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double back_left_velocity = motor_controller_->readThenWriteVelocity(
                                    MotorIndex::BACK_LEFT, back_left_target_rpm) *
                                MECHANICAL_MPS_PER_ELECTRICAL_RPM;
    double dribbler_rpm = motor_controller_->readThenWriteVelocity(MotorIndex::DRIBBLER,
                                                                   dribbler_ramp_rpm_);

    // Construct a MotorStatus object with the current velocities and dribbler rpm
    TbotsProto::MotorStatus motor_status =
        updateMotorStatus(front_left_velocity, front_right_velocity, back_left_velocity,
                          back_right_velocity, dribbler_rpm);

    // This order needs to match euclidean_to_four_wheel converters order
    // We also want to work in the meters per second space rather than electrical RPMs
    WheelSpace_t current_wheel_velocities = {front_right_velocity, front_left_velocity,
                                             back_left_velocity, back_right_velocity};

    // if (std::abs(current_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] -
    //              prev_wheel_velocities_[FRONT_RIGHT_WHEEL_SPACE_INDEX]) >
    //     RUNAWAY_PROTECTION_THRESHOLD_MPS)
    // {
    //     motor_controller_->immediatelyDisable();
    //     LOG(FATAL) << "Front right motor runaway";
    // }
    // else if (std::abs(current_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] -
    //                   prev_wheel_velocities_[FRONT_LEFT_WHEEL_SPACE_INDEX]) >
    //          RUNAWAY_PROTECTION_THRESHOLD_MPS)
    // {
    //     motor_controller_->immediatelyDisable();
    //     LOG(FATAL) << "Front left motor runaway";
    // }
    // else if (std::abs(current_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] -
    //                   prev_wheel_velocities_[BACK_LEFT_WHEEL_SPACE_INDEX]) >
    //          RUNAWAY_PROTECTION_THRESHOLD_MPS)
    // {
    //     motor_controller_->immediatelyDisable();
    //     LOG(FATAL) << "Back left motor runaway";
    // }
    // else if (std::abs(current_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] -
    //                   prev_wheel_velocities_[BACK_RIGHT_WHEEL_SPACE_INDEX]) >
    //          RUNAWAY_PROTECTION_THRESHOLD_MPS)
    // {
    //     motor_controller_->immediatelyDisable();
    //     LOG(FATAL) << "Back right motor runaway";
    // }
    (void)RUNAWAY_PROTECTION_THRESHOLD_MPS;

    // Convert to Euclidean velocity_delta
    EuclideanSpace_t current_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(current_wheel_velocities);

    motor_status.mutable_local_velocity()->set_x_component_meters(
        current_euclidean_velocity[1]);
    motor_status.mutable_local_velocity()->set_y_component_meters(
        -current_euclidean_velocity[0]);
    motor_status.mutable_angular_velocity()->set_radians_per_second(
        current_euclidean_velocity[2]);

    WheelSpace_t target_wheel_velocities = WheelSpace_t::Zero();

    // Get target wheel velocities from the primitive
    if (motor.has_direct_per_wheel_control())
    {
        TbotsProto::MotorControl_DirectPerWheelControl direct_per_wheel =
            motor.direct_per_wheel_control();
        target_wheel_velocities = {
            direct_per_wheel.front_right_wheel_velocity(),
            direct_per_wheel.front_left_wheel_velocity(),
            direct_per_wheel.back_left_wheel_velocity(),
            direct_per_wheel.back_right_wheel_velocity(),
        };
    }
    else if (motor.has_direct_velocity_control())
    {
        TbotsProto::MotorControl_DirectVelocityControl direct_velocity =
            motor.direct_velocity_control();
        EuclideanSpace_t target_euclidean_velocity = {
            -direct_velocity.velocity().y_component_meters(),
            direct_velocity.velocity().x_component_meters(),
            direct_velocity.angular_velocity().radians_per_second()};

        target_wheel_velocities =
            euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity);
    }

    // ramp the target velocities to keep acceleration compared to current velocities
    // within safe bounds
    target_wheel_velocities = euclidean_to_four_wheel_.rampWheelVelocity(
        prev_wheel_velocities_, target_wheel_velocities, time_elapsed_since_last_poll_s);

    prev_wheel_velocities_ = target_wheel_velocities;

    // Calculate speeds accounting for acceleration
    front_right_target_rpm =
        static_cast<int>(target_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    front_left_target_rpm =
        static_cast<int>(target_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    back_left_target_rpm =
        static_cast<int>(target_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);
    back_right_target_rpm =
        static_cast<int>(target_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX] *
                         ELECTRICAL_RPM_PER_MECHANICAL_MPS);

    // Get target dribbler rpm from the primitive
    int target_dribbler_rpm;
    if (motor.drive_control_case() ==
        TbotsProto::MotorControl::DriveControlCase::DRIVE_CONTROL_NOT_SET)
    {
        target_dribbler_rpm = 0;
    }
    else
    {
        target_dribbler_rpm = motor.dribbler_speed_rpm();
    }

    // Ramp the dribbler velocity
    // Clamp the max acceleration
    int max_dribbler_delta_rpm = static_cast<int>(
        DRIBBLER_ACCELERATION_THRESHOLD_RPM_PER_S_2 * time_elapsed_since_last_poll_s);
    int delta_rpm = std::clamp(target_dribbler_rpm - dribbler_ramp_rpm_,
                               -max_dribbler_delta_rpm, max_dribbler_delta_rpm);
    dribbler_ramp_rpm_ += delta_rpm;

    // Clamp to the max rpm
    int max_dribbler_rpm =
        std::abs(static_cast<int>(robot_constants_.max_force_dribbler_speed_rpm));
    dribbler_ramp_rpm_ =
        std::clamp(dribbler_ramp_rpm_, -max_dribbler_rpm, max_dribbler_rpm);

    motor_status.mutable_dribbler()->set_dribbler_rpm(float(dribbler_ramp_rpm_));

    return motor_status;
}

bool MotorService::requiresMotorReinit(const MotorIndex& motor)
{
    auto reset_search =
        cached_motor_faults_[motor].motor_faults.find(TbotsProto::MotorFault::RESET);

    return !cached_motor_faults_[motor].drive_enabled ||
           (reset_search != cached_motor_faults_[motor].motor_faults.end());
}
