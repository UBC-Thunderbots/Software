#include "software/ai/navigator/navigator.h"

#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"

Navigator::Navigator(std::unique_ptr<PathManager> path_manager,
                     RobotNavigationObstacleFactory robot_navigation_obstacle_factory,
                     std::shared_ptr<const NavigatorConfig> config)
    : config(config),
      robot_navigation_obstacle_factory(std::move(robot_navigation_obstacle_factory)),
      path_manager(std::move(path_manager))
{
}

std::unique_ptr<PrimitiveSetMsg> Navigator::getAssignedPrimitiveSetMsg(
    const World &world, const std::vector<std::unique_ptr<Intent>> &intents)
{
    planned_paths.clear();
    Rectangle navigable_area = world.field().fieldBoundary();
    auto path_objectives     = createPathObjectives(intents, world);
    auto robot_id_to_path =
        path_manager->getManagedPaths(path_objectives, navigable_area);
    return createPrimitives(intents, world, robot_id_to_path);
}

std::optional<PathObjective> Navigator::createPathObjective(
    RobotId robot_id, const NavigatorParams &navigator_params, const World &world)
{
    std::vector<ObstaclePtr> obstacles =
        robot_navigation_obstacle_factory.createFromMotionConstraints(
            navigator_params.motion_constraints, world);

    if (navigator_params.ball_collision_type == BallCollisionType::AVOID)
    {
        const auto ball_obstacle =
            robot_navigation_obstacle_factory.createFromBallPosition(
                world.ball().position());
        obstacles.push_back(ball_obstacle);
    }

    auto robot = world.friendlyTeam().getRobotById(robot_id);
    if (robot)
    {
        return PathObjective{.robot_id      = robot_id,
                             .start         = robot->position(),
                             .end           = navigator_params.destination,
                             .current_speed = robot->velocity().length(),
                             .obstacles     = obstacles};
    }
    else
    {
        LOG(WARNING) << "Failed to find robot associated with robot id = " << robot_id;
        return std::nullopt;
    }
}

std::vector<PathObjective> Navigator::createPathObjectives(
    const std::vector<std::unique_ptr<Intent>> &intents, const World &world)
{
    std::vector<ObstaclePtr> friendly_non_navigating_robot_obstacles;
    std::vector<PathObjective> path_objectives;

    for (const auto &intent : intents)
    {
        auto navigator_params = intent->getNavigatorParams();
        if (navigator_params)
        {
            auto path_objective =
                createPathObjective(intent->getRobotId(), *navigator_params, world);
            if (path_objective)
            {
                path_objectives.emplace_back(*path_objective);
            }
        }
        else
        {
            auto robot = world.friendlyTeam().getRobotById(intent->getRobotId());
            if (robot)
            {
                auto robot_obstacle =
                    robot_navigation_obstacle_factory.createFromRobot(*robot);
                friendly_non_navigating_robot_obstacles.push_back(robot_obstacle);
            }
        }
    }

    for (auto &path_objective : path_objectives)
    {
        path_objective.obstacles.insert(path_objective.obstacles.end(),
                                        friendly_non_navigating_robot_obstacles.begin(),
                                        friendly_non_navigating_robot_obstacles.end());
    }
    return path_objectives;
}

std::unique_ptr<PrimitiveSetMsg> Navigator::createPrimitives(
    const std::vector<std::unique_ptr<Intent>> &intents, const World &world,
    std::map<RobotId, std::optional<Path>> robot_id_to_path)
{
    auto primitive_set_msg                    = std::make_unique<PrimitiveSetMsg>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestampMsg();
    auto &robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();

    for (const auto &intent : intents)
    {
        auto navigator_params = intent->getNavigatorParams();
        if (navigator_params)
        {
            auto robot_id_to_path_iter = robot_id_to_path.find(intent->getRobotId());
            if (robot_id_to_path_iter != robot_id_to_path.end() &&
                robot_id_to_path_iter->second)
            {
                auto [destination, final_speed] = calculateDestinationAndFinalSpeed(
                    navigator_params.value(), *(robot_id_to_path_iter->second), world);
                robot_primitives_map[intent->getRobotId()] =
                    navigator_params->primitive_msg_update_function(destination,
                                                                    final_speed);
            }
            else
            {
                LOG(WARNING)
                    << "Navigator's path manager could not find a path for RobotId = "
                    << intent->getRobotId();
                robot_primitives_map[intent->getRobotId()] =
                    ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
                        StopPrimitive(intent->getRobotId(), false));
            }
        }
        else
        {
            robot_primitives_map[intent->getRobotId()] = intent->getPrimitiveMsg();
        }
    }
    return primitive_set_msg;
}

std::pair<Point, double> Navigator::calculateDestinationAndFinalSpeed(
    const NavigatorParams &navigator_params, const Path &path, const World &world)
{
    double desired_final_speed;
    Point final_dest;
    std::vector<Point> path_points = path.getKnots();
    planned_paths.emplace_back(path_points);

    if (path_points.size() <= 2)
    {
        // we are going to destination
        desired_final_speed = navigator_params.final_speed;
        final_dest          = path.getEndPoint();
    }
    else
    {
        // we are going to some intermediate point so we transition smoothly
        double transition_final_speed =
            ROBOT_MAX_SPEED_METERS_PER_SECOND * config->TransitionSpeedFactor()->value();

        desired_final_speed = calculateTransitionSpeedBetweenSegments(
            path_points[0], path_points[1], path_points[2], transition_final_speed);

        final_dest = path_points[1];
    }

    return std::make_pair<Point, double>(
        Point(final_dest),  // slow down around enemy robots
        desired_final_speed *
            getEnemyObstacleProximityFactor(path_points[1], world.enemyTeam()));
}

double Navigator::getEnemyObstacleProximityFactor(const Point &p, const Team &enemy_team)
{
    double robot_proximity_limit = config->EnemyRobotProximityLimit()->value();

    // find min dist between p and any robot
    double closest_dist = std::numeric_limits<double>::max();
    auto obstacles      = robot_navigation_obstacle_factory.createFromTeam(enemy_team);
    for (const auto &obstacle : obstacles)
    {
        double current_dist = obstacle->distance(p);
        if (current_dist < closest_dist)
        {
            closest_dist = current_dist;
        }
    }

    // clamp ratio between 0 and 1
    return std::clamp(closest_dist / robot_proximity_limit, 0.0, 1.0);
}

double Navigator::calculateTransitionSpeedBetweenSegments(const Point &p1,
                                                          const Point &p2,
                                                          const Point &p3,
                                                          double final_speed)
{
    return final_speed * (p2 - p1).normalize().project((p3 - p2).normalize()).length();
}

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}

std::vector<ObstaclePtr> Navigator::getObstacles()
{
    return path_manager->getObstacles();
}
