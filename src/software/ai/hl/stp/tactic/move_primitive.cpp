#include "software/ai/hl/stp/tactic/move_primitive.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d_angular.h"

MovePrimitive::MovePrimitive(
    const Robot &robot, const Point &destination, const Angle &final_angle,
    const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
    const TbotsProto::DribblerMode &dribbler_mode,
    const TbotsProto::BallCollisionType &ball_collision_type,
    const AutoChipOrKick &auto_chip_or_kick, std::optional<double> cost_override)
    : robot(robot),
      destination(destination),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      auto_chip_or_kick(auto_chip_or_kick),
      ball_collision_type(ball_collision_type),
      max_allowed_speed_mode(max_allowed_speed_mode)
{
    if (cost_override.has_value())
    {
        estimated_cost = cost_override.value();
    }
    else
    {
        double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(
            max_allowed_speed_mode, robot.robotConstants());
        trajectory.generate(robot.position(), destination, robot.velocity(), max_speed,
                            robot.robotConstants().robot_max_acceleration_m_per_s_2,
                            robot.robotConstants().robot_max_deceleration_m_per_s_2);

        angular_trajectory.generate(
            robot.orientation(), final_angle, robot.angularVelocity(),
            AngularVelocity::fromRadians(
                robot.robotConstants().robot_max_ang_speed_rad_per_s),
            AngularVelocity::fromRadians(
                robot.robotConstants().robot_max_ang_acceleration_rad_per_s_2),
            AngularVelocity::fromRadians(
                robot.robotConstants().robot_max_ang_acceleration_rad_per_s_2));

        estimated_cost =
            std::max(trajectory.getTotalTime(), angular_trajectory.getTotalTime());
    }
}

std::unique_ptr<TbotsProto::Primitive> MovePrimitive::generatePrimitiveProtoMessage(
    const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    auto primitive_proto = std::make_unique<TbotsProto::Primitive>();

    // Generate obstacle avoiding trajectory
    std::vector<ObstaclePtr> obstacles =
        generateObstacles(world, motion_constraints, obstacle_factory);

    double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(
        max_allowed_speed_mode, robot.robotConstants());
    KinematicConstraints constraints(
        max_speed, robot.robotConstants().robot_max_acceleration_m_per_s_2,
        robot.robotConstants().robot_max_deceleration_m_per_s_2);

    // TODO: Instead of field boundary, it should be made smaller 9cm
    TrajectoryPath traj_path =
        planner.findTrajectory(robot.position(), destination, robot.velocity(),
                               constraints, obstacles, world.field().fieldBoundary());

    // Populate the move primitive proto with the trajectory path parameters
    TbotsProto::TrajectoryPathParams2D xy_traj_params;
    *(xy_traj_params.mutable_start_position())   = *createPointProto(robot.position());
    *(xy_traj_params.mutable_destination())      = *createPointProto(destination);
    *(xy_traj_params.mutable_initial_velocity()) = *createVectorProto(robot.velocity());
    xy_traj_params.set_max_speed_mode(max_allowed_speed_mode);
    *(primitive_proto->mutable_move()->mutable_xy_traj_params()) = xy_traj_params;

    TbotsProto::TrajectoryParamsAngular1D w_traj_params;
    *(w_traj_params.mutable_start_angle()) = *createAngleProto(robot.orientation());
    *(w_traj_params.mutable_final_angle()) = *createAngleProto(final_angle);
    *(w_traj_params.mutable_initial_velocity()) =
        *createAngularVelocityProto(robot.angularVelocity());
    w_traj_params.set_constraints_multiplier(1.0);
    *(primitive_proto->mutable_move()->mutable_w_traj_params()) = w_traj_params;

    primitive_proto->mutable_move()->set_dribbler_mode(dribbler_mode);

    if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOCHIP)
    {
        primitive_proto->mutable_move()
            ->mutable_auto_chip_or_kick()
            ->set_autochip_distance_meters(
                static_cast<float>(auto_chip_or_kick.autochip_distance_m));
    }
    else if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOKICK)
    {
        primitive_proto->mutable_move()
            ->mutable_auto_chip_or_kick()
            ->set_autokick_speed_m_per_s(
                static_cast<float>(auto_chip_or_kick.autokick_speed_m_per_s));
    }

    const auto &path_nodes = traj_path.getTrajectoryPathNodes();
    *(primitive_proto->mutable_move()
          ->mutable_xy_traj_params()
          ->mutable_sub_destination()) =
        *createPointProto(path_nodes[0].getTrajectory()->getDestination());

    // TODO (NIMA): Consider improving this logic
    if (path_nodes[0].getTrajectoryEndTime() !=
        path_nodes[0].getTrajectory()->getTotalTime())
    {
        primitive_proto->mutable_move()->mutable_xy_traj_params()->set_connection_time(
            static_cast<float>(path_nodes[0].getTrajectoryEndTime()));
    }
    else
    {
        primitive_proto->mutable_move()->mutable_xy_traj_params()->set_connection_time(0);
    }

    return std::move(primitive_proto);
}

std::vector<ObstaclePtr> MovePrimitive::generateObstacles(
    const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const RobotNavigationObstacleFactory &obstacle_factory) const
{
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createObstaclesFromMotionConstraints(motion_constraints, world);

    for (const Robot &enemy : world.enemyTeam().getAllRobots())
    {
        obstacles.push_back(obstacle_factory.createFromRobotPosition(enemy.position()));
    }

    for (const Robot &friendly : world.friendlyTeam().getAllRobots())
    {
        if (friendly.id() != robot.id())
        {
            obstacles.push_back(
                obstacle_factory.createFromRobotPosition(friendly.position()));
        }
    }

    if (ball_collision_type == TbotsProto::AVOID)
    {
        obstacles.push_back(
            obstacle_factory.createFromBallPosition(world.ball().position()));
    }

    return obstacles;
}
