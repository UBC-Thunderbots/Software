#include "software/ai/navigator/navigator.h"

#include "software/ai/navigator/navigating_primitive_creator.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/proto/primitive/primitive_msg_factory.h"

Navigator::Navigator(std::unique_ptr<PathManager> path_manager,
                     RobotNavigationObstacleFactory robot_navigation_obstacle_factory,
                     std::shared_ptr<const NavigatorConfig> config)
    : config(config),
      robot_navigation_obstacle_factory(std::move(robot_navigation_obstacle_factory)),
      path_manager(std::move(path_manager))
{
}

void Navigator::visit(const DirectPrimitiveIntent &intent)
{
    (*primitive_set_msg->mutable_robot_primitives())[intent.getRobotId()] =
        intent.getPrimitive();
    direct_primitive_intent_robots.push_back(intent.getRobotId());
}

void Navigator::visit(const MoveIntent &intent)
{
    navigating_intents.push_back(std::make_shared<MoveIntent>(intent));
}

std::unique_ptr<TbotsProto::PrimitiveSet> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &intents)
{
    // Initialize variables
    navigating_intents.clear();
    planned_paths.clear();
    direct_primitive_intent_robots.clear();
    primitive_set_msg = std::make_unique<TbotsProto::PrimitiveSet>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestamp();

    // Register all intents
    for (const auto &intent : intents)
    {
        intent->accept(*this);
    }

    // Plan paths
    Rectangle navigable_area = world.field().fieldBoundary();
    auto path_objectives     = createPathObjectives(world);
    auto robot_id_to_path =
        path_manager->getManagedPaths(path_objectives, navigable_area);

    // Add primitives from navigating intents
    auto &robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();
    for (const auto &intent : navigating_intents)
    {
        unsigned int robot_id      = intent->getRobotId();
        auto robot_id_to_path_iter = robot_id_to_path.find(robot_id);
        if (robot_id_to_path_iter != robot_id_to_path.end() &&
            robot_id_to_path_iter->second)
        {
            planned_paths.push_back(robot_id_to_path_iter->second->getKnots());
            robot_primitives_map[robot_id] =
                NavigatingPrimitiveCreator(config).createNavigatingPrimitive(
                    *intent, *(robot_id_to_path_iter->second),
                    robot_navigation_obstacle_factory.createFromTeam(world.enemyTeam()));
        }
        else
        {
            LOG(WARNING)
                << "Navigator's path manager could not find a path for RobotId = "
                << robot_id;
            robot_primitives_map[robot_id] = *createStopPrimitive(false);
        }
    }

    return std::move(primitive_set_msg);
}

std::unordered_set<PathObjective> Navigator::createPathObjectives(
    const World &world) const
{
    std::unordered_set<PathObjective> path_objectives;
    std::vector<ObstaclePtr> direct_primitive_intent_obstacles;
    auto ball_obstacle =
        robot_navigation_obstacle_factory.createFromBallPosition(world.ball().position());

    for (const auto &robot_id : direct_primitive_intent_robots)
    {
        auto robot = world.friendlyTeam().getRobotById(robot_id);
        if (robot)
        {
            auto robot_obstacle =
                robot_navigation_obstacle_factory.createFromRobot(*robot);
            direct_primitive_intent_obstacles.push_back(robot_obstacle);
        }
        else
        {
            std::stringstream ss;
            ss << "Failed to find robot associated with robot id = " << robot_id;
            LOG(WARNING) << ss.str();
        }
    }

    for (const auto &intent : navigating_intents)
    {
        RobotId robot_id = intent->getRobotId();
        // start with direct primitive intent robots and then add motion constraints
        auto obstacles = direct_primitive_intent_obstacles;

        auto robot = world.friendlyTeam().getRobotById(robot_id);

        if (robot)
        {
            auto motion_constraint_obstacles =
                robot_navigation_obstacle_factory.createFromMotionConstraints(
                    intent->getMotionConstraints(), world);
            obstacles.insert(obstacles.end(), motion_constraint_obstacles.begin(),
                             motion_constraint_obstacles.end());

            std::vector<ObstaclePtr> enemy_robot_obstacles =
                robot_navigation_obstacle_factory.createEnemyCollisionAvoidance(
                    world.enemyTeam(), robot->velocity().length());
            obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                             enemy_robot_obstacles.end());

            if (intent->getBallCollisionType() == BallCollisionType::AVOID)
            {
                obstacles.push_back(ball_obstacle);
            }

            Point start = robot->position();
            Point end   = intent->getDestination();

            path_objectives.insert(PathObjective(start, end, robot->velocity().length(),
                                                 obstacles, robot_id));
        }
        else
        {
            std::stringstream ss;
            ss << "Failed to find robot associated with robot id = " << robot_id;
            LOG(WARNING) << ss.str();
        }
    }
    return path_objectives;
}

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}

std::vector<ObstaclePtr> Navigator::getObstacles()
{
    return path_manager->getObstacles();
}
