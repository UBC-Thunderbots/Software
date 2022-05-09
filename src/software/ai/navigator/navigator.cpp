#include "software/ai/navigator/navigator.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/visualization.pb.h"
#include "software/ai/navigator/navigating_primitive_creator.h"
#include "software/geom/algorithms/distance.h"
#include "software/gui/drawing/obstacle_artist.h"
#include "software/logger/logger.h"

Navigator::Navigator(std::unique_ptr<PathManager> path_manager,
                     RobotNavigationObstacleFactory robot_navigation_obstacle_factory)
    : robot_navigation_obstacle_factory(std::move(robot_navigation_obstacle_factory)),
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

    TbotsProto::PathVisualization path_visualization_proto;

    // Add primitives from navigating intents
    auto &robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();
    for (const auto &intent : navigating_intents)
    {
        unsigned int robot_id      = intent->getRobotId();
        auto robot_id_to_path_iter = robot_id_to_path.find(robot_id);
        if (robot_id_to_path_iter != robot_id_to_path.end() &&
            robot_id_to_path_iter->second)
        {
            auto knots = robot_id_to_path_iter->second->getKnots();
            planned_paths.push_back(knots);

            TbotsProto::Path path_proto;

            for (auto knot : knots)
            {
                *(path_proto.add_point()) = *createPointProto(knot);
            }

            robot_primitives_map[robot_id] =
                NavigatingPrimitiveCreator().createNavigatingPrimitive(
                    *intent, *(robot_id_to_path_iter->second),
                    robot_navigation_obstacle_factory.createFromTeam(world.enemyTeam()));

            *(path_visualization_proto.add_path()) = path_proto;
        }
        else
        {
            LOG(WARNING)
                << "Navigator's path manager could not find a path for RobotId = "
                << robot_id;
            robot_primitives_map[robot_id] = *createStopPrimitive(false);
        }
    }

    LOG(VISUALIZE) << path_visualization_proto;

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

            if (intent->getBallCollisionType() == TbotsProto::BallCollisionType::AVOID)
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

    // TODO (#2584) This is a nasty hack we need to visualize the obstacles
    // using the obstacle artist.
    //
    // We will be visualizing static obstacles
    // through primitives and dynamic obstacles through the HRVO layer so
    // we can remove this hack.
    ObstacleArtist obstacle_artist(NULL, std::nullopt);
    for (const auto &obstacle : getObstacles())
    {
        obstacle->accept(obstacle_artist);
    }
    obstacle_artist.visualize();

    return path_objectives;
}

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}

std::vector<ObstaclePtr> Navigator::getObstacles() const
{
    return path_manager->getObstacles();
}
