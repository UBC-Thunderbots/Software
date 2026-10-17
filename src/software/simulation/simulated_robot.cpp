#include "software/simulation/simulated_robot.h"

#include <cmath>

#include "proto/message_translation/tbots_geometry.h"
#include "shared/constants.h"
#include "software/embedded/services/imu.h"

namespace
{
// Datasheet gyroscope noise, which the robot localizer also assumes
SimulatedImuService::NoiseConfig realisticImuNoise()
{
    return {.angular_velocity_stddev_rad_per_s = std::sqrt(ImuService::IMU_VARIANCE)};
}

// Standard deviation of rounding wheel speeds to whole RPM, as the motor drivers do
SimulatedMotorService::NoiseConfig realisticMotorNoise(
    const robot_constants::RobotConstants& robot_constants)
{
    const double one_rpm_m_per_s = 2 * M_PI * robot_constants.wheel_radius_meters / 60;
    return {.wheel_velocity_stddev_m_per_s = one_rpm_m_per_s / std::sqrt(12.0)};
}
}  // namespace

SimulatedRobot::SimulatedRobot(const RobotId robot_id,
                               const robot_constants::RobotConstants& robot_constants,
                               const bool enable_realism)
    : motor_service_(std::make_unique<SimulatedMotorService>(
          robot_constants,
          enable_realism ? realisticMotorNoise(robot_constants)
                         : SimulatedMotorService::NoiseConfig(),
          robot_id)),
      imu_service_(std::make_unique<SimulatedImuService>(
          enable_realism ? realisticImuNoise() : SimulatedImuService::NoiseConfig(),
          robot_id)),
      robot_localizer_(enable_realism ? std::make_unique<RobotLocalizer>() : nullptr),
      primitive_executor_(std::make_unique<PrimitiveExecutor>(robot_constants, robot_id))
{
}

void SimulatedRobot::setPrimitive(const TbotsProto::Primitive& primitive,
                                  const RobotState& true_state)
{
    if (robot_localizer_)
    {
        updateRobotLocalizer(primitive);
    }
    else
    {
        primitive_executor_->updateRobotState(true_state);
    }
    primitive_executor_->updatePrimitive(primitive, robot_status_);
}

TbotsProto::DirectControlPrimitive SimulatedRobot::tick(const RobotState& true_state,
                                                        const Duration& time_step)
{
    // Same order as Thunderloop::runLoop
    if (robot_localizer_)
    {
        robot_localizer_->predict(primitive_executor_->getPrevCommandedVelocity(),
                                  time_step);
        updateRobotLocalizer(robot_status_);
        primitive_executor_->updateRobotState(robot_localizer_->getRobotState());
    }

    imu_service_->poll(true_state, robot_status_, time_step);

    const TbotsProto::DirectControlPrimitive direct_control_primitive =
        primitive_executor_->stepPrimitive(robot_status_, time_step);

    return motor_service_->poll(direct_control_primitive, true_state, robot_status_,
                                time_step);
}

void SimulatedRobot::reset()
{
    imu_service_->reset();
    if (robot_localizer_)
    {
        robot_localizer_ = std::make_unique<RobotLocalizer>();
    }
    robot_status_.Clear();
}

void SimulatedRobot::updateRobotLocalizer(const TbotsProto::Primitive& primitive)
{
    if (primitive.has_move())
    {
        const Point position =
            createPoint(primitive.move().xy_traj_params().start_position());
        const Angle orientation =
            createAngle(primitive.move().w_traj_params().start_angle());
        robot_localizer_->update(
            RobotLocalizer::VisionData{position, orientation, RTT_S / 2});
    }
}

void SimulatedRobot::updateRobotLocalizer(const TbotsProto::RobotStatus& robot_status)
{
    // Separate update is okay because measurement model is linear
    if (robot_status.has_imu_status())
    {
        robot_localizer_->update(RobotLocalizer::ImuData{
            createAngularVelocity(robot_status.imu_status().angular_velocity())});
    }
    if (robot_status.has_motor_status())
    {
        const Vector velocity =
            createVector(robot_status.motor_status().local_velocity());
        const AngularVelocity angular_velocity =
            createAngularVelocity(robot_status.motor_status().angular_velocity());
        robot_localizer_->update(RobotLocalizer::MotorData{velocity, angular_velocity});
    }
}
