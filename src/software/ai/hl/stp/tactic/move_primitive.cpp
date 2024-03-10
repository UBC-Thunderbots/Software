#include "software/ai/hl/stp/tactic/move_primitive.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/geom/algorithms/end_in_obstacle_sample.h"

MovePrimitive::MovePrimitive(
    const Robot &robot, const Point &destination, const Angle &final_angle,
    const TbotsProto::MaxAllowedSpeedMode &max_allowed_speed_mode,
    const TbotsProto::ObstacleAvoidanceMode &obstacle_avoidance_mode,
    const TbotsProto::DribblerMode &dribbler_mode,
    const TbotsProto::BallCollisionType &ball_collision_type,
    const AutoChipOrKick &auto_chip_or_kick, std::optional<double> cost_override)
    : robot(robot),
      destination(destination),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      auto_chip_or_kick(auto_chip_or_kick),
      ball_collision_type(ball_collision_type),
      max_allowed_speed_mode(max_allowed_speed_mode),
      obstacle_avoidance_mode(obstacle_avoidance_mode)
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

std::pair<std::optional<TrajectoryPath>, std::unique_ptr<TbotsProto::Primitive>>
MovePrimitive::generatePrimitiveProtoMessage(
    const WorldPtr &world_ptr,
    const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const std::map<RobotId, TrajectoryPath> &robot_trajectories,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    // Generate obstacle avoiding trajectory
    updateObstacles(world_ptr, motion_constraints, robot_trajectories, obstacle_factory);

    double max_speed = convertMaxAllowedSpeedModeToMaxAllowedSpeed(
        max_allowed_speed_mode, robot.robotConstants());
    KinematicConstraints constraints(
        max_speed, robot.robotConstants().robot_max_acceleration_m_per_s_2,
        robot.robotConstants().robot_max_deceleration_m_per_s_2);

    // TODO (#3104): The fieldBounary should be shrunk by the robot radius before being
    //  passed to the planner.
    Rectangle navigable_area = world_ptr->field().fieldBoundary();

    // If the robot is in a static obstacle, then we should first move to the nearest
    // point out
    std::optional<Point> updated_start_position =
        endInObstacleSample(field_obstacles, robot.position(), navigable_area);
    if (updated_start_position.has_value() &&
        updated_start_position.value() != robot.position())
    {
        destination = updated_start_position.value();
    }
    else
    {
        std::optional<Point> updated_destination =
            endInObstacleSample(field_obstacles, destination, navigable_area);
        if (updated_destination.has_value())
        {
            // Update the destination. Note that this may be the same as the original
            // destination.
            destination = updated_destination.value();
        }
        else
        {
            LOG(WARNING) << "Could not move the destination for robot " << robot.id()
                         << " from " << destination
                         << " to a point outside of the field obstacles.";
        }
    }

    std::optional<Point> prev_sub_destination;
    auto prev_trajectory_it = robot_trajectories.find(robot.id());
    if (prev_trajectory_it != robot_trajectories.end())
    {
        const auto &prev_trajectory_path_nodes =
            prev_trajectory_it->second.getTrajectoryPathNodes();
        if (!prev_trajectory_path_nodes.empty())
        {
            prev_sub_destination =
                prev_trajectory_path_nodes[0].getTrajectory()->getDestination();
        }
    }

    traj_path = planner.findTrajectory(robot.position(), destination, robot.velocity(),
                                       constraints, obstacles, navigable_area,
                                       prev_sub_destination);

    if (!traj_path.has_value())
    {
        LOG(WARNING) << "Could not find trajectory path for robot " << robot.id()
                     << " to move to " << destination;
        return std::make_pair(std::nullopt, std::move(createStopPrimitiveProto()));
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
    // Populate the sub-destinations if there are any
    if (path_nodes.size() >= 2)
    {
        // The last path node goes to the destination which is stored above
        for (unsigned int i = 0; i < path_nodes.size() - 1; ++i)
        {
            TbotsProto::TrajectoryPathParams2D::SubDestination sub_destination_proto;
            *(sub_destination_proto.mutable_sub_destination()) =
                *createPointProto(path_nodes[i].getTrajectory()->getDestination());
            sub_destination_proto.set_connection_time_s(
                static_cast<float>(path_nodes[i].getTrajectoryEndTime()));
            *(primitive_proto->mutable_move()
                  ->mutable_xy_traj_params()
                  ->add_sub_destinations()) = sub_destination_proto;
        }
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

    return std::make_pair(traj_path, std::move(primitive_proto));
}

void MovePrimitive::updateObstacles(
    const WorldPtr &world_ptr,
    const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const std::map<RobotId, TrajectoryPath> &robot_trajectories,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    // Separately store the non-robot + non-ball obstacles
    field_obstacles = obstacle_factory.createObstaclesFromMotionConstraints(
        motion_constraints, world_ptr);

    obstacles = field_obstacles;

    for (const Robot &enemy : world_ptr->enemyTeam().getAllRobots())
    {
        if (obstacle_avoidance_mode == TbotsProto::SAFE)
        {
            // Generate a possibly long stadium shape obstacle in the region
            // where the enemy robot may move in
            obstacles.push_back(obstacle_factory.createStadiumEnemyRobotObstacle(enemy));
        }
        else if (obstacle_avoidance_mode == TbotsProto::AGGRESSIVE)
        {
            // Generate a moving obstacle depending on the enemy robot's velocity.
            // This is considered a more aggressive strategy as it assumes the enemy
            // robot is moving at a constant speed. The generated obstacle can also be
            // much smaller than the stadium shape obstacle, allowing the robot to move
            // more freely.
            obstacles.push_back(
                obstacle_factory.createConstVelocityEnemyRobotObstacle(enemy));
        }
    }

    for (const Robot &friendly : world_ptr->friendlyTeam().getAllRobots())
    {
        if (friendly.id() != robot.id())
        {
            auto traj_iter = robot_trajectories.find(friendly.id());
            if (traj_iter != robot_trajectories.end())
            {
                obstacles.push_back(
                    obstacle_factory.createFromMovingRobot(friendly, traj_iter->second));
            }
            else
            {
                obstacles.push_back(
                    obstacle_factory.createStaticObstacleFromRobotPosition(
                        friendly.position()));
            }
        }
    }

    if (ball_collision_type == TbotsProto::AVOID)
    {
        obstacles.push_back(
            obstacle_factory.createFromBallPosition(world_ptr->ball().position()));
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
