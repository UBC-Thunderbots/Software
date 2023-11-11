#include "proto/primitive/primitive_msg_factory.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"

std::unique_ptr<TbotsProto::Primitive>
createMovePrimitive(const Robot &robot,
                    const Point &destination,
                    const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
                    const Angle &final_angle,
                    const TbotsProto::DribblerMode &dribbler_mode,
                    const TbotsProto::BallCollisionType &ball_collision_type,
                    const AutoChipOrKick &auto_chip_or_kick,
                    const RobotConstants_t &robot_constants,
                    std::optional<double> cost_override)
{
    auto traj_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    TbotsProto::TrajectoryPathParams2D xy_traj_params;
    *(xy_traj_params.mutable_start_position()) = *createPointProto(robot.position());
    *(xy_traj_params.mutable_destination()) = *createPointProto(destination);
    *(xy_traj_params.mutable_initial_velocity()) = *createVectorProto(robot.velocity());
    xy_traj_params.set_max_speed_mode(max_allowed_speed_mode);
    *(traj_primitive_msg->mutable_move()->mutable_xy_traj_params()) = xy_traj_params;

    TbotsProto::TrajectoryParamsAngular1D w_traj_params;
    *(w_traj_params.mutable_start_angle()) = *createAngleProto(robot.orientation());
    *(w_traj_params.mutable_final_angle()) = *createAngleProto(final_angle);
    *(w_traj_params.mutable_initial_velocity()) = *createAngularVelocityProto(robot.angularVelocity());
    w_traj_params.set_constraints_multiplier(1.0);
    *(traj_primitive_msg->mutable_move()->mutable_w_traj_params()) = w_traj_params;

    traj_primitive_msg->mutable_move()->set_dribbler_mode(dribbler_mode);

    if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOCHIP)
    {
        traj_primitive_msg->mutable_move()
                ->mutable_auto_chip_or_kick()
                ->set_autochip_distance_meters(
                        static_cast<float>(auto_chip_or_kick.autochip_distance_m));
    }
    else if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOKICK)
    {
        traj_primitive_msg->mutable_move()
                ->mutable_auto_chip_or_kick()
                ->set_autokick_speed_m_per_s(
                        static_cast<float>(auto_chip_or_kick.autokick_speed_m_per_s));
    }

    traj_primitive_msg->mutable_move()->set_ball_collision_type(ball_collision_type);

    // TODO (NIMA): Use the trajectory factory to calculate the cost of the trajectory. Note that no path is needed for direct trajectory
    if (cost_override.has_value())
    {
        traj_primitive_msg->set_cost(cost_override.value());
    }
    else
    {
        double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(
                max_allowed_speed_mode, robot_constants);
        BangBangTrajectory2D trajectory(robot.position(),
                                        destination,
                                        robot.velocity(),
                                        max_speed,
                                        robot_constants.robot_max_acceleration_m_per_s_2,
                                        robot_constants.robot_max_deceleration_m_per_s_2);

        // TODO: Combine generate and constructor
        BangBangTrajectory1DAngular angular_trajectory;
        angular_trajectory.generate(robot.orientation(),
                                    final_angle,
                                    robot.angularVelocity(),
                                    AngularVelocity::fromRadians(robot_constants.robot_max_ang_speed_rad_per_s),
                                    AngularVelocity::fromRadians(robot_constants.robot_max_ang_acceleration_rad_per_s_2),
                                    AngularVelocity::fromRadians(robot_constants.robot_max_ang_acceleration_rad_per_s_2));

        traj_primitive_msg->set_cost(std::max(trajectory.getTotalTime(), angular_trajectory.getTotalTime()));
    }
    return traj_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createStopPrimitive()
{
    auto stop_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    stop_primitive_msg->mutable_stop();

    stop_primitive_msg->set_cost(1.0);

    return stop_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createDirectControlPrimitive(
    const Vector &velocity, AngularVelocity angular_velocity, double dribbler_speed_rpm,
    const TbotsProto::AutoChipOrKick &auto_chip_or_kick)
{
    auto direct_control_primitive_msg = std::make_unique<TbotsProto::Primitive>();
    auto direct_velocity_control =
        std::make_unique<TbotsProto::MotorControl::DirectVelocityControl>();

    *(direct_velocity_control->mutable_velocity()) = *createVectorProto(velocity);
    *(direct_velocity_control->mutable_angular_velocity()) =
        *createAngularVelocityProto(angular_velocity);

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_motor_control()
          ->mutable_direct_velocity_control()) = *direct_velocity_control;

    direct_control_primitive_msg->mutable_direct_control()
        ->mutable_motor_control()
        ->set_dribbler_speed_rpm(static_cast<float>(dribbler_speed_rpm));

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_power_control()
          ->mutable_chicker()
          ->mutable_auto_chip_or_kick()) = auto_chip_or_kick;
    return direct_control_primitive_msg;
}

double convertDribblerModeToDribblerSpeed(TbotsProto::DribblerMode dribbler_mode,
                                          RobotConstants_t robot_constants)
{
    switch (dribbler_mode)
    {
        case TbotsProto::DribblerMode::INDEFINITE:
            return robot_constants.indefinite_dribbler_speed_rpm;
        case TbotsProto::DribblerMode::MAX_FORCE:
            return robot_constants.max_force_dribbler_speed_rpm;
        case TbotsProto::DribblerMode::OFF:
            return 0.0;
        default:
            LOG(WARNING) << "DribblerMode is invalid" << std::endl;
            return 0.0;
    }
}

double convertMaxAllowedSpeedModeToMaxAllowedSpeed(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    RobotConstants_t robot_constants)
{
    switch (max_allowed_speed_mode)
    {
        case TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT:
            return robot_constants.robot_max_speed_m_per_s;
        case TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND:
            return STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND;
        case TbotsProto::MaxAllowedSpeedMode::COLLISIONS_ALLOWED:
            return COLLISION_ALLOWED_ROBOT_MAX_SPEED_METERS_PER_SECOND;
        default:
            LOG(WARNING) << "MaxAllowedSpeedMode is invalid" << std::endl;
            return 0.0;
    }
}
