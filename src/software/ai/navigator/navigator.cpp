#include "software/ai/navigator/navigator.h"

#include "software/logger/logger.h"
#include "software/new_geom/util/distance.h"

Navigator::Navigator(std::unique_ptr<PathManager> path_manager,
                     ObstacleFactory obstacle_factory,
                     std::shared_ptr<const NavigatorConfig> config)
    : config(config),
      obstacle_factory(std::move(obstacle_factory)),
      path_manager(std::move(path_manager))

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
    move_intents_for_path_planning.push_back(intent);
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
    move_intents_for_path_planning.clear();
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

    for (auto &mi_primitive :
         getPrimitivesFromMoveIntents(move_intents_for_path_planning))
    {
        assigned_primitives.emplace_back(std::move(mi_primitive));
    }

    return assigned_primitives;
}

std::unordered_set<PathObjective> Navigator::getPathObjectivesFromMoveIntents(
    const std::vector<MoveIntent> &move_intents)
{
    std::unordered_set<PathObjective> path_objectives;
    for (const auto &intent : move_intents)
    {
        // start with non-MoveIntent robots and then add motion constraints
        auto obstacles = friendly_non_move_intent_robot_obstacles;

        auto motion_constraint_obstacles =
            obstacle_factory.createObstaclesFromMotionConstraints(
                intent.getMotionConstraints(), world);
        obstacles.insert(obstacles.end(), motion_constraint_obstacles.begin(),
                         motion_constraint_obstacles.end());

        if (intent.getBallCollisionType() == BallCollisionType::AVOID)
        {
            auto ball_obstacle =
                obstacle_factory.createBallObstacle(world.ball().position());
            obstacles.push_back(ball_obstacle);
        }

        auto robot = world.friendlyTeam().getRobotById(intent.getRobotId());

        if (robot)
        {
            Point start = robot->position();
            Point end   = intent.getDestination();

            path_objectives.insert(PathObjective(start, end, robot->velocity().length(),
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

std::vector<std::unique_ptr<Primitive>> Navigator::getPrimitivesFromMoveIntents(
    const std::vector<MoveIntent> &move_intents)
{
    std::vector<std::unique_ptr<Primitive>> primitives;

    Rectangle navigable_area = this->world.field().fieldBoundary();

    auto path_objectives = getPathObjectivesFromMoveIntents(move_intents);

    auto robot_id_to_path =
        path_manager->getManagedPaths(path_objectives, navigable_area);

    // Turn each intent and associated path into primitives
    for (const auto &intent : move_intents)
    {
        std::unique_ptr<Primitive> primitive;
        if (robot_id_to_path.find(intent.getRobotId()) == robot_id_to_path.end())
        {
            LOG(WARNING) << "Path manager did not map RobotId = " << intent.getRobotId()
                         << " to a path";
            // generate primitive from no path
            primitive = getPrimitiveFromPathAndMoveIntent(std::nullopt, intent);
        }
        else
        {
            auto path = robot_id_to_path.at(intent.getRobotId());
            primitive = getPrimitiveFromPathAndMoveIntent(path, intent);
        }
        primitives.emplace_back(std::move(primitive));
    }
    return primitives;
}

void Navigator::registerNonMoveIntentRobotId(RobotId id)
{
    auto robot = world.friendlyTeam().getRobotById(id);
    if (robot)
    {
        auto robot_obstacle = obstacle_factory.createVelocityObstacleFromRobot(*robot);
        friendly_non_move_intent_robot_obstacles.push_back(robot_obstacle);
    }
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
                                            config->TransitionSpeedFactor()->value();

            desired_final_speed = calculateTransitionSpeedBetweenSegments(
                path_points[0], path_points[1], path_points[2], transition_final_speed);

            final_dest = path_points[1];
        }

        return std::make_unique<MovePrimitive>(
            intent.getRobotId(), final_dest, intent.getFinalAngle(),
            // slow down around enemy robots
            desired_final_speed *
                getEnemyObstacleProximityFactor(path_points[1], world.enemyTeam()),
            intent.getDribblerEnable(), intent.getMoveType(), intent.getAutoKickType());
    }
    else
    {
        LOG(WARNING) << "Path manager could not find a path for RobotId = "
                     << intent.getRobotId();
        return std::make_unique<StopPrimitive>(intent.getRobotId(), false);
    }
}

double Navigator::getEnemyObstacleProximityFactor(const Point &p, const Team &enemy_team)
{
    double robot_proximity_limit = config->EnemyRobotProximityLimit()->value();

    // find min dist between p and any robot
    double closest_dist = std::numeric_limits<double>::max();
    auto obstacles      = obstacle_factory.createVelocityObstaclesFromTeam(enemy_team);
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
