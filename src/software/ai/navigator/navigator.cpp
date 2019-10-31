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
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const ChipIntent &intent)
{
    auto p            = std::make_unique<ChipPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const DirectVelocityIntent &intent)
{
    auto p            = std::make_unique<DirectVelocityPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const DirectWheelsIntent &intent)
{
    auto p            = std::make_unique<DirectWheelsPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const DribbleIntent &intent)
{
    auto p            = std::make_unique<DribblePrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const KickIntent &intent)
{
    auto p            = std::make_unique<KickPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const MoveIntent &intent)
{
    Point start =
        this->world.friendlyTeam().getRobotById(intent.getRobotId())->position();
    Point end                 = intent.getDestination();
    auto avoid_area_obstacles = getObstaclesFromAvoidAreas(intent.getAreasToAvoid());

    auto robot = world.friendlyTeam().getRobotById(intent.getRobotId());

    if (robot)
    {
        path_objective_to_move_intent.insert(
            {PathObjective(start, end, robot->velocity().len(), avoid_area_obstacles,
                           intent.getRobotId()),
             intent});
    }
    else
    {
        std::stringstream ss;
        ss << "Tried to find robot associated with robot id = " << intent.getRobotId()
           << ", but failed";
        LOG(WARNING) << ss.str();
    }
    current_primitive = std::unique_ptr<Primitive>(nullptr);
}

void Navigator::visit(const MoveSpinIntent &intent)
{
    auto p            = std::make_unique<MoveSpinPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const PivotIntent &intent)
{
    auto p            = std::make_unique<PivotPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

void Navigator::visit(const StopIntent &intent)
{
    auto p            = std::make_unique<StopPrimitive>(intent);
    current_primitive = std::move(p);
    non_path_planning_robots.insert(intent.getRobotId());
}

std::vector<std::unique_ptr<Primitive>> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world              = world;
    planned_paths.clear();
    non_path_planning_robots.clear();
    path_objective_to_move_intent.clear();

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (current_primitive)
        {
            assigned_primitives.emplace_back(std::move(current_primitive));
        }
    }

    Rectangle navigable_area(
        Point(this->world.field().totalXLength(), this->world.field().totalYLength()),
        this->world.field().totalXLength(), this->world.field().totalYLength());

    // get keyset from path_objective_to_move_intent
    std::set<PathObjective> path_objectives;
    for (const auto &p : path_objective_to_move_intent)
    {
        path_objectives.insert(p.first);
    }
    auto paths = path_manager->getManagedPaths(path_objectives, navigable_area,
                                               getNonPathPlanningObstacles());
    addPathsToAssignedPrimitives(paths, assigned_primitives);

    return assigned_primitives;
}

void Navigator::addPathsToAssignedPrimitives(
    const std::map<PathObjective, std::optional<Path>> &path_objective_to_path,
    std::vector<std::unique_ptr<Primitive>> &assigned_primitives)
{
    for (const auto &path_objective_and_path : path_objective_to_path)
    {
        auto path_objective = path_objective_and_path.first;
        auto path           = path_objective_and_path.second;

        // look for move intent
        auto path_objective_and_intent_it =
            path_objective_to_move_intent.find(path_objective);
        if (path_objective_and_intent_it == path_objective_to_move_intent.end())
        {
            std::stringstream ss;
            ss << "Tried to find intent associated with path objective that has robot id = "
               << path_objective.robot_id << ", but failed";
            LOG(WARNING) << ss.str();

            // assume no path
            continue;
        }

        MoveIntent intent = path_objective_and_intent_it->second;

        if (path)
        {
            std::vector<Point> path_points = path->getKnots();
            planned_paths.emplace_back(path_points);

            if (path_points.size() == 1)
            {
                throw std::runtime_error(
                    "Path only contains one point, which is invalid, since it's ambiguous if it's the start or end or some other point");
            }
            else
            {
                double desired_final_speed;

                if (path_points.size() == 2)
                {
                    // we are going to destination
                    desired_final_speed = intent.getFinalSpeed();
                }
                else
                {
                    // we are going to some intermediate point so we transition smoothly
                    double transition_final_speed =
                        ROBOT_MAX_SPEED_METERS_PER_SECOND *
                        Util::DynamicParameters->getNavigatorConfig()
                            ->TransitionSpeedFactor()
                            ->value();

                    desired_final_speed = calculateTransitionSpeedBetweenSegments(
                        path_points[0], path_points[1], path_points[2],
                        transition_final_speed);
                }

                auto move = std::make_unique<MovePrimitive>(
                    intent.getRobotId(), path_points[1], intent.getFinalAngle(),
                    // slow down around enemy robots
                    desired_final_speed * getCloseToEnemyObstacleFactor(path_points[1]),
                    intent.getDribblerEnable(), intent.getMoveType(),
                    intent.getAutoKickType());
                current_primitive = std::move(move);
            }
        }
        else
        {
            LOG(WARNING) << "Path manager could not find a path";
            auto stop = std::make_unique<StopPrimitive>(intent.getRobotId(), false);
            current_primitive = std::move(stop);
        }
        assigned_primitives.emplace_back(std::move(current_primitive));
    }
}

std::vector<Obstacle> Navigator::getNonPathPlanningObstacles()
{
    std::vector<Obstacle> obstacles;
    double inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                  ->RobotObstacleInflationFactor()
                                  ->value();
    for (auto &robot : world.friendlyTeam().getAllRobots())
    {
        if (non_path_planning_robots.find(robot.id()) != non_path_planning_robots.end())
        {
            obstacles.push_back(
                Obstacle::createCircularRobotObstacle(robot, inflation_factor));
        }
    }
    return obstacles;
}

std::vector<Obstacle> Navigator::getObstaclesFromAvoidAreas(
    const std::vector<AvoidArea> &avoid_areas)
{
    std::vector<Obstacle> obstacles;
    Rectangle rectangle({0, 0}, {0, 0});
    for (auto avoid_area : avoid_areas)
    {
        switch (avoid_area)
        {
            case AvoidArea::ENEMY_ROBOTS:
            {
                std::vector<Obstacle> enemy_robot_obstacles =
                    getObstaclesFromTeam(world.enemyTeam());
                obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                                 enemy_robot_obstacles.end());
            }
            break;
            case AvoidArea::FRIENDLY_DEFENSE_AREA:
                // We extend the friendly defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().friendlyDefenseArea().posXPosYCorner(),
                    Point(-10, world.field().friendlyDefenseArea().posXNegYCorner().y()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::ENEMY_DEFENSE_AREA:
                // We extend the enemy defense area back by several meters to prevent
                // robots going around the back of the goal
                rectangle = Rectangle(
                    world.field().enemyDefenseArea().negXPosYCorner(),
                    Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::INFLATED_ENEMY_DEFENSE_AREA:
                rectangle = world.field().enemyDefenseArea();
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                         ->RobotObstacleInflationFactor()
                                         ->value() *
                                     ROBOT_MAX_RADIUS_METERS +
                                 0.3);  // 0.3 is by definition what inflated means
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::CENTER_CIRCLE:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.field().centerPoint(), world.field().centerCircleRadius(),
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case AvoidArea::HALF_METER_AROUND_BALL:
                obstacles.push_back(Obstacle::createCircleObstacle(
                    world.ball().position(), 0.5,  // 0.5 represents half a metre radius
                    Util::DynamicParameters->getNavigatorConfig()
                        ->RobotObstacleInflationFactor()
                        ->value()));
                break;
            case AvoidArea::BALL:
                obstacles.push_back(
                    Obstacle::createCircularBallObstacle(world.ball(), 0.06));
                break;
            case AvoidArea::ENEMY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().enemyCornerNeg() -
                                          Point(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            case AvoidArea::FRIENDLY_HALF:
                rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                      world.field().friendlyCornerNeg() -
                                          Point(0, world.field().boundaryYLength()));
                rectangle.expand(Util::DynamicParameters->getNavigatorConfig()
                                     ->RobotObstacleInflationFactor()
                                     ->value() *
                                 ROBOT_MAX_RADIUS_METERS);
                obstacles.push_back(Obstacle(rectangle));
                break;
            default:
                LOG(WARNING) << "Could not convert AvoidArea " << (int)avoid_area
                             << " to obstacle";
        }
    }

    return obstacles;
}

double Navigator::getCloseToEnemyObstacleFactor(const Point &p)
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

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}

std::vector<Obstacle> Navigator::getObstaclesFromTeam(const Team &team)
{
    double robot_inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                        ->RobotObstacleInflationFactor()
                                        ->value();
    double velocity_inflation_factor = Util::DynamicParameters->getNavigatorConfig()
                                           ->VelocityObstacleInflationFactor()
                                           ->value();
    std::vector<Obstacle> obstacles;
    for (auto &robot : team.getAllRobots())
    {
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(
            robot, robot_inflation_factor, velocity_inflation_factor);
        obstacles.push_back(o);
    }
    return obstacles;
}
