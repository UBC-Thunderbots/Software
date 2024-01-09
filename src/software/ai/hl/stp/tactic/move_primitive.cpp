#include "software/ai/hl/stp/tactic/move_primitive.h"

#include <tracy/Tracy.hpp>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"

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
    ZoneScopedN("MovePrimitive::generatePrimitiveProtoMessage");
    // Generate obstacle avoiding trajectory
    generateObstacles(world, motion_constraints, obstacle_factory);

    double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(
        max_allowed_speed_mode, robot.robotConstants());
    KinematicConstraints constraints(
        max_speed, robot.robotConstants().robot_max_acceleration_m_per_s_2,
        robot.robotConstants().robot_max_deceleration_m_per_s_2);

    // TODO (#3104): The fieldBounary should be shrunk by the robot radius before being
    //  passed to the planner.
    traj_path =
        planner.findTrajectory(robot.position(), destination, robot.velocity(),
                               constraints, obstacles, world.field().fieldBoundary());

    if (!traj_path.has_value())
    {
        LOG(WARNING) << "Could not find trajectory path for robot " << robot.id()
                     << " to move to " << destination;
        return createStopPrimitiveProto();
    }

    estimated_cost = traj_path->getTotalTime();

    // Populate the move primitive proto with the trajectory path parameters
    auto primitive_proto = std::make_unique<TbotsProto::Primitive>();

    TbotsProto::TrajectoryPathParams2D xy_traj_params;
    *(xy_traj_params.mutable_start_position())   = *createPointProto(robot.position());
    *(xy_traj_params.mutable_destination())      = *createPointProto(destination);
    *(xy_traj_params.mutable_initial_velocity()) = *createVectorProto(robot.velocity());
    xy_traj_params.set_max_speed_mode(max_allowed_speed_mode);
    *(primitive_proto->mutable_move()->mutable_xy_traj_params()) = xy_traj_params;

    const auto &path_nodes = traj_path->getTrajectoryPathNodes();
    // Set sub_destination and connection_time_s fields, if the trajectory path
    // consists of more than 1 trajectory
    if (path_nodes.size() >= 2)
    {
        *(primitive_proto->mutable_move()
              ->mutable_xy_traj_params()
              ->mutable_sub_destination()) =
            *createPointProto(path_nodes[0].getTrajectory()->getDestination());

        primitive_proto->mutable_move()->mutable_xy_traj_params()->set_connection_time_s(
            static_cast<float>(path_nodes[0].getTrajectoryEndTime()));
    }
    else
    {
        // If the trajectory path consists of only 1 trajectory,
        // then the connection time is set to 0
        primitive_proto->mutable_move()->mutable_xy_traj_params()->set_connection_time_s(
            0);
    }

    TbotsProto::TrajectoryParamsAngular1D w_traj_params;
    *(w_traj_params.mutable_start_angle()) = *createAngleProto(robot.orientation());
    *(w_traj_params.mutable_final_angle()) = *createAngleProto(final_angle);
    *(w_traj_params.mutable_initial_velocity()) =
        *createAngularVelocityProto(robot.angularVelocity());
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

    return primitive_proto;
}

void MovePrimitive::generateObstacles(
    const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    obstacles =
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
}

void MovePrimitive::getVisualizationProtos(
    TbotsProto::ObstacleList &obstacle_list_out,
    TbotsProto::PathVisualization &path_visualization_out) const
{
    for (const auto &obstacle : obstacles)
    {
        obstacle_list_out.add_obstacles()->CopyFrom(obstacle->createObstacleProto());
    }

    TbotsProto::Path path;
    if (traj_path.has_value())
    {
        for (unsigned int i = 0; i < NUM_TRAJECTORY_VISUALIZATION_POINTS; i++)
        {
            double t =
                i * traj_path->getTotalTime() / (NUM_TRAJECTORY_VISUALIZATION_POINTS - 1);
            Point position = traj_path->getPosition(t);
            path.add_points()->CopyFrom(*createPointProto(position));
        }
    }
    path_visualization_out.add_paths()->CopyFrom(path);
}
