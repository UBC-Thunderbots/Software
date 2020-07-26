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
    const World &world, const std::vector<std::unique_ptr<Intent>> &assigned_intents)
{
    planned_paths.clear();

    std::vector<ObstaclePtr> friendly_non_navigating_robot_obstacles;

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assigned_intents)
    {
        auto navigator_params = intent->getNavigatorParams();
        if (!navigator_params)
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

    Rectangle navigable_area = world.field().fieldBoundary();
    auto path_objectives     = getPathObjectivesFromIntents(
        assigned_intents, world, friendly_non_navigating_robot_obstacles);
    auto robot_id_to_path =
        path_manager->getManagedPaths(path_objectives, navigable_area);

    // create msg and update timestamp
    auto primitive_set_msg = std::make_unique<PrimitiveSetMsg>();
    primitive_set_msg->set_timestamp_seconds(createCurrentTime());

    // set robot primitives
    auto &robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();

    // Turn each intent and associated path into primitives
    for (const auto &intent : assigned_intents)
    {
        auto navigator_params = intent->getNavigatorParams();
        if (navigator_params)
        {
            if (robot_id_to_path.find(navigator_params->robot_id) !=
                robot_id_to_path.end())
            {
                auto path = robot_id_to_path.at(navigator_params->robot_id);
                auto [destination, final_speed] = calculateDestinationAndFinalSpeed(
                    navigator_params.value(), *path, intent, world);

                robot_primitives_map[intent->getRobotId()] =
                    intent->getPrimitiveMsg(destination, final_speed);
            }
            else
            {
                LOG(WARNING) << "Path manager did not map RobotId = "
                             << navigator_params->robot_id << " to a path";
                robot_primitives_map[intent->getRobotId()] = intent->getPrimitiveMsg();
            }
        }
        else
        {
            robot_primitives_map[intent->getRobotId()] = intent->getPrimitiveMsg();
        }
    }

    return primitive_set_msg;
}

std::vector<std::unique_ptr<Primitive>> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assigned_intents)
{
    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    return assigned_primitives;
}

std::unordered_set<PathObjective> Navigator::getPathObjectivesFromIntents(
    const std::vector<std::unique_ptr<Intent>> &intents, const World &world,
    std::vector<ObstaclePtr> friendly_non_navigating_robot_obstacles)
{
    std::unordered_set<PathObjective> path_objectives;

    for (const auto &intent : intents)
    {
        auto navigator_params = intent->getNavigatorParams();
        if (navigator_params)
        {
            // start with non-MoveIntent robots and then add motion constraints
            auto obstacles = friendly_non_navigating_robot_obstacles;

            auto motion_constraint_obstacles =
                robot_navigation_obstacle_factory.createFromMotionConstraints(
                    navigator_params->motion_constraints, world);
            obstacles.insert(obstacles.end(), motion_constraint_obstacles.begin(),
                             motion_constraint_obstacles.end());

            if (navigator_params->ball_collision_type == BallCollisionType::AVOID)
            {
                auto ball_obstacle =
                    robot_navigation_obstacle_factory.createFromBallPosition(
                        world.ball().position());
                obstacles.push_back(ball_obstacle);
            }

            auto robot = world.friendlyTeam().getRobotById(navigator_params->robot_id);

            if (robot)
            {
                Point start = robot->position();
                Point end   = navigator_params->destination;

                path_objectives.insert(
                    PathObjective(start, end, robot->velocity().length(), obstacles,
                                  navigator_params->robot_id));
            }
            else
            {
                std::stringstream ss;
                ss << "Failed to find robot associated with robot id = "
                   << navigator_params->robot_id;
                LOG(WARNING) << ss.str();
            }
        }
    }
    return path_objectives;
}

std::pair<Point, double> Navigator::calculateDestinationAndFinalSpeed(
    NavigatorParams navigator_params, Path path, const std::unique_ptr<Intent> &intent,
    const World &world)
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
