#include "software/ai/navigator/navigator.h"

#include <g3log/g3log.hpp>

Navigator::Navigator(std::unique_ptr<PathManager> path_manager)
    : path_manager(std::move(path_manager))
{
}

void Navigator::visit(const CatchIntent &intent)
{
    auto p            = std::make_unique<CatchPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const ChipIntent &intent)
{
    auto p            = std::make_unique<ChipPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const DirectVelocityIntent &intent)
{
    auto p            = std::make_unique<DirectVelocityPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const DirectWheelsIntent &intent)
{
    auto p            = std::make_unique<DirectWheelsPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const DribbleIntent &intent)
{
    auto p            = std::make_unique<DribblePrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const KickIntent &intent)
{
    auto p            = std::make_unique<KickPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const MoveIntent &intent)
{
    move_intents.push_back(intent);
    current_primitive = std::unique_ptr<Primitive>(nullptr);
}

void Navigator::visit(const MoveSpinIntent &intent)
{
    auto p            = std::make_unique<MoveSpinPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const PivotIntent &intent)
{
    auto p            = std::make_unique<PivotPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

void Navigator::visit(const StopIntent &intent)
{
    auto p            = std::make_unique<StopPrimitive>(intent);
    current_primitive = std::move(p);
    registerNonMoveIntentRobotId(intent.getRobotId());
}

std::vector<std::unique_ptr<Primitive>> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world = world;
    planned_paths.clear();
    move_intents.clear();
    friendly_non_move_intent_robot_obstacles.clear();

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (current_primitive)
        {
            assigned_primitives.emplace_back(std::move(current_primitive));
        }
    }

    addMoveIntentsToAssignedPrimitives(assigned_primitives);

    return assigned_primitives;
}

std::set<PathObjective> Navigator::generatePathObjectives(void)
{
    std::set<PathObjective> path_objectives;
    for (const auto &intent : move_intents)
    {
        // start with non-MoveIntent robots and then add avoid areas
        auto obstacles = friendly_non_move_intent_robot_obstacles;
        auto avoid_area_obstacles =
            getObstaclesFromAvoidAreas(intent.getAreasToAvoid(), world);
        obstacles.insert(obstacles.end(), avoid_area_obstacles.begin(),
                         avoid_area_obstacles.end());

        auto robot = world.friendlyTeam().getRobotById(intent.getRobotId());

        if (robot)
        {
            Point start = robot->position();
            Point end   = intent.getDestination();

            path_objectives.insert(PathObjective(start, end, robot->velocity().len(),
                                                 obstacles, intent.getRobotId()));
        }
        else
        {
            std::stringstream ss;
            ss << "Failed to find robot associated with robot id = "
               << intent.getRobotId();
            LOG(WARNING) << ss.str();
        }
    }
    return path_objectives;
}

void Navigator::addMoveIntentsToAssignedPrimitives(
    std::vector<std::unique_ptr<Primitive>> &assigned_primitives)
{
    Rectangle navigable_area = this->world.field().fieldBoundary();

    auto path_objectives = generatePathObjectives();

    auto robot_id_to_path =
        path_manager->getManagedPaths(path_objectives, navigable_area);

    // Turn each intent and associated path into primitives
    for (const auto &intent : move_intents)
    {
        auto path      = robot_id_to_path[intent.getRobotId()];
        auto primitive = getPrimitiveFromPathAndMoveIntent(path, intent);
        assigned_primitives.emplace_back(std::move(primitive));
    }
}

void Navigator::registerNonMoveIntentRobotId(RobotId id)
{
    double inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                  ->RobotObstacleInflationFactor()
                                  ->value();
    auto robot = (world.friendlyTeam().getRobotById(id));
    if (robot)
    {
        friendly_non_move_intent_robot_obstacles.push_back(
            Obstacle::createCircularRobotObstacle(*robot, inflation_factor));
    }
}

double Navigator::getEnemyObstacleProximityFactor(const Point &p)
{
    // find min dist between p and any robot
    double closest_dist = DBL_MAX;
    for (auto &robot : world.enemyTeam().getAllRobots())
    {
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(
            robot,
            Util::DynamicParameters->getNavigatorConfig()
                ->RobotObstacleInflationFactor()
                ->value(),
            Util::DynamicParameters->getNavigatorConfig()
                ->VelocityObstacleInflationFactor()
                ->value());
        double current_dist = dist(p, (*o.getBoundaryPolygon()));
        if (current_dist < closest_dist)
        {
            closest_dist = current_dist;
        }
    }

    // clamp ratio between 0 and 1
    return std::clamp(closest_dist / ROBOT_MAX_SPEED_METERS_PER_SECOND, 0.0, 1.0);
}

std::unique_ptr<Primitive> Navigator::getPrimitiveFromPathAndMoveIntent(
    std::optional<Path> path, MoveIntent intent)
{
    if (path)
    {
        double desired_final_speed;
        Point final_dest;
        std::vector<Point> path_points = path->getKnots();
        planned_paths.emplace_back(path_points);

        if (path_points.size() <= 2)
        {
            // we are going to destination
            desired_final_speed = intent.getFinalSpeed();
            final_dest          = path->endPoint();
        }
        else
        {
            // we are going to some intermediate point so we transition smoothly
            double transition_final_speed = ROBOT_MAX_SPEED_METERS_PER_SECOND *
                                            Util::DynamicParameters->getNavigatorConfig()
                                                ->TransitionSpeedFactor()
                                                ->value();

            desired_final_speed = calculateTransitionSpeedBetweenSegments(
                path_points[0], path_points[1], path_points[2], transition_final_speed);

            final_dest = path_points[1];
        }

        return std::make_unique<MovePrimitive>(
            intent.getRobotId(), final_dest, intent.getFinalAngle(),
            // slow down around enemy robots
            desired_final_speed * getEnemyObstacleProximityFactor(path_points[1]),
            intent.getDribblerEnable(), intent.getMoveType(), intent.getAutoKickType());
    }
    else
    {
        LOG(WARNING) << "Path manager could not find a path";
        return std::make_unique<StopPrimitive>(intent.getRobotId(), false);
    }
}

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}
