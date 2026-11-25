#include "software/simulation/simulated_robot.h"

#include "proto/message_translation/tbots_geometry.h"
#include "shared/constants.h"


SimulatedRobot::SimulatedRobot(RobotId robot_id,
                               const robot_constants::RobotConstants& robot_constants,
                               bool enable_robot_localizer,
                               const SimulatedRobotSensorNoise& sensor_noise)
    : primitive_executor_(std::make_unique<PrimitiveExecutor>(robot_constants, robot_id))
{
    const SimulatedMotorService::NoiseConfig motor_noise{
        .wheel_velocity_stddev_m_per_s =
            sensor_noise.motor().wheel_velocity_stddev_m_per_s()};
    motor_service_ =
        std::make_unique<SimulatedMotorService>(robot_constants, motor_noise, robot_id);

    const SimulatedImuService::NoiseConfig imu_noise{
        .angular_velocity_bias_rad_per_s =
            sensor_noise.imu().angular_velocity_bias_rad_per_s(),
        .angular_velocity_stddev_rad_per_s =
            sensor_noise.imu().angular_velocity_stddev_rad_per_s(),
        .linear_acceleration_bias_m_per_s_2 =
            Vector(sensor_noise.imu().linear_acceleration_bias_x_m_per_s_2(),
                   sensor_noise.imu().linear_acceleration_bias_y_m_per_s_2()),
        .linear_acceleration_stddev_m_per_s_2 =
            sensor_noise.imu().linear_acceleration_stddev_m_per_s_2()};
    imu_service_ = std::make_unique<SimulatedImuService>(imu_noise, robot_id);

    if (enable_robot_localizer)
    {
        robot_localizer_ = std::make_unique<RobotLocalizer>();
    }
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
