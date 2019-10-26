#include "software/ai/navigator/navigator.h"

#include <g3log/g3log.hpp>

#include "software/ai/intent/all_intents.h"
#include "software/ai/navigator/util.h"
#include "software/ai/primitive/all_primitives.h"

Navigator::Navigator(std::unique_ptr<PathPlanner> path_planner)
    : path_planner(std::move(path_planner))
{
}

void Navigator::visit(const CatchIntent &catch_intent)
{
    auto p            = std::make_unique<CatchPrimitive>(catch_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const ChipIntent &chip_intent)
{
    auto p            = std::make_unique<ChipPrimitive>(chip_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const DirectVelocityIntent &direct_velocity_intent)
{
    auto p            = std::make_unique<DirectVelocityPrimitive>(direct_velocity_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const DirectWheelsIntent &direct_wheels_intent)
{
    auto p            = std::make_unique<DirectWheelsPrimitive>(direct_wheels_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const DribbleIntent &dribble_intent)
{
    auto p            = std::make_unique<DribblePrimitive>(dribble_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const KickIntent &kick_intent)
{
    auto p            = std::make_unique<KickPrimitive>(kick_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const MoveIntent &move_intent)
{
    current_start =
        this->world.friendlyTeam().getRobotById(move_intent.getRobotId())->position();
    current_end = move_intent.getDestination();

    std::vector<Obstacle> obstacles =
        getCurrentObstacles(move_intent.getAreasToAvoid(), move_intent.getRobotId());

    Rectangle navigable_area(
        Point(this->world.field().totalXLength(), this->world.field().totalYLength()),
        this->world.field().totalXLength(), this->world.field().totalYLength());

    Path path =
        path_planner->findPath(current_start, current_end, navigable_area, obstacles);

    moveNavigation(move_intent, path);
}

void Navigator::visit(const MoveSpinIntent &move_spin_intent)
{
    auto p            = std::make_unique<MoveSpinPrimitive>(move_spin_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const PivotIntent &pivot_intent)
{
    auto p            = std::make_unique<PivotPrimitive>(pivot_intent);
    current_primitive = std::move(p);
}

void Navigator::visit(const StopIntent &stop_intent)
{
    auto p            = std::make_unique<StopPrimitive>(stop_intent);
    current_primitive = std::move(p);
}

// helpers
void Navigator::moveNavigation(const MoveIntent &move_intent, const Path &path)
{
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
            current_destination = path_points[1];

            if (path_points.size() == 2)
            {
                // we are going to destination
                desired_final_speed = move_intent.getFinalSpeed();
            }
            else
            {
                // we are going to some intermediate point so we transition smoothly
                double transition_final_speed =
                    ROBOT_MAX_SPEED_METERS_PER_SECOND *
                    Util::DynamicParameters::Navigator::transition_speed_factor.value();

                desired_final_speed = calculateTransitionSpeedBetweenSegments(
                    path_points[0], current_destination, path_points[2],
                    transition_final_speed);
            }

            auto move = std::make_unique<MovePrimitive>(
                move_intent.getRobotId(), current_destination,
                move_intent.getFinalAngle(),
                // slow down around enemy robots
                desired_final_speed * getCloseToEnemyObstacleFactor(current_destination),
                move_intent.getDribblerEnable(), move_intent.getMoveType(),
                move_intent.getAutoKickType());
            current_primitive = std::move(move);
        }
    }
    else
    {
        LOG(WARNING) << "Path planner could not find a path";
        auto stop = std::make_unique<StopPrimitive>(move_intent.getRobotId(), false);
        current_primitive = std::move(stop);
    }
}

std::optional<Obstacle> Navigator::getObstacleFromAvoidArea(AvoidArea avoid_area)
{
    Rectangle rectangle({0, 0}, {0, 0});
    switch (avoid_area)
    {
        case AvoidArea::FRIENDLY_DEFENSE_AREA:
            // We extend the friendly defense area back by several meters to prevent
            // robots going around the back of the goal
            rectangle = Rectangle(
                world.field().friendlyDefenseArea().posXPosYCorner(),
                Point(-10, world.field().friendlyDefenseArea().posXNegYCorner().y()));
            rectangle.expand(
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value() *
                ROBOT_MAX_RADIUS_METERS);
            return Obstacle(rectangle);
        case AvoidArea::ENEMY_DEFENSE_AREA:
            // We extend the enemy defense area back by several meters to prevent
            // robots going around the back of the goal
            rectangle = Rectangle(
                world.field().enemyDefenseArea().negXPosYCorner(),
                Point(10, world.field().enemyDefenseArea().negXNegYCorner().y()));
            rectangle.expand(
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value() *
                ROBOT_MAX_RADIUS_METERS);
            return Obstacle(rectangle);
        case AvoidArea::INFLATED_ENEMY_DEFENSE_AREA:
            rectangle = world.field().enemyDefenseArea();
            rectangle.expand(
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                        .value() *
                    ROBOT_MAX_RADIUS_METERS +
                0.3);  // 0.3 is by definition what inflated means
            return Obstacle(rectangle);
        case AvoidArea::CENTER_CIRCLE:
            return Obstacle::createCircleObstacle(
                world.field().centerPoint(), world.field().centerCircleRadius(),
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value());
        case AvoidArea::HALF_METER_AROUND_BALL:
            return Obstacle::createCircleObstacle(
                world.ball().position(), 0.5,  // 0.5 represents half a metre radius
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value());
        case AvoidArea::BALL:
            return Obstacle::createCircularBallObstacle(world.ball(), 0.06);
        case AvoidArea::ENEMY_HALF:
            rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                  world.field().enemyCornerNeg() -
                                      Point(0, world.field().boundaryYLength()));
            rectangle.expand(
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value() *
                ROBOT_MAX_RADIUS_METERS);
            return Obstacle(rectangle);
        case AvoidArea::FRIENDLY_HALF:
            rectangle = Rectangle({0, world.field().totalYLength() / 2},
                                  world.field().friendlyCornerNeg() -
                                      Point(0, world.field().boundaryYLength()));
            rectangle.expand(
                Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                    .value() *
                ROBOT_MAX_RADIUS_METERS);
            return Obstacle(rectangle);
        default:
            LOG(WARNING) << "Could not convert AvoidArea " << (int)avoid_area
                         << " to obstacle";
    }

    return std::nullopt;
}

std::vector<std::unique_ptr<Primitive>> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world              = world;
    this->current_robot      = std::nullopt;
    this->current_velocity_obstacles = {};
    planned_paths.clear();

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (this->current_robot)
        {
            if (this->current_robot->velocity().len() > 0.3)
            {
                this->current_velocity_obstacles.emplace_back(
                    Obstacle::createVelocityObstacleWithScalingParams(
                        this->current_robot->position(), this->current_destination,
                        this->current_robot->velocity().len(),
                        Util::DynamicParameters::Navigator::
                            robot_obstacle_inflation_factor.value(),
                        Util::DynamicParameters::Navigator::
                            velocity_obstacle_inflation_factor.value()));
            }

            this->current_robot = std::nullopt;
        }
        assigned_primitives.emplace_back(std::move(current_primitive));
    }

    return assigned_primitives;
}

std::vector<Obstacle> Navigator::getCurrentObstacles(
    const std::vector<AvoidArea> &avoid_areas, unsigned int robot_id)
{
    std::vector<Obstacle> obstacles = current_velocity_obstacles;

    // Avoid obstacles specific to this MoveIntent
    for (auto area : avoid_areas)
    {
        if (area == AvoidArea::ENEMY_ROBOTS)
        {
            for (auto &robot : world.enemyTeam().getAllRobots())
            {
                Obstacle o = Obstacle::createRobotObstacleWithScalingParams(
                    robot,
                    Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor
                        .value(),
                    Util::DynamicParameters::Navigator::velocity_obstacle_inflation_factor
                        .value());
                obstacles.push_back(o);
            }
        }
        else
        {
            auto obstacle_opt = getObstacleFromAvoidArea(area);
            if (obstacle_opt)
            {
                obstacles.emplace_back(*obstacle_opt);
            }
        }
    }

    for (auto &robot : world.friendlyTeam().getAllRobots())
    {
        if (robot.id() == robot_id)
        {
            // store current robot
            this->current_robot = robot;
            // skip current robot
            continue;
        }
        Obstacle o = Obstacle::createCircularRobotObstacle(
            robot,
            Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor.value());
        obstacles.push_back(o);
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
            Util::DynamicParameters::Navigator::robot_obstacle_inflation_factor.value(),
            Util::DynamicParameters::Navigator::velocity_obstacle_inflation_factor
                .value());
        double current_dist = dist(p, (*o.getBoundaryPolygon()));
        if (current_dist < closest_dist)
        {
            closest_dist = current_dist;
        }
    }

    // clamp ratio between 0 and 1
    return std::clamp(closest_dist / ROBOT_MAX_SPEED_METERS_PER_SECOND, 0.0, 1.0);
}

std::vector<std::vector<Point>> Navigator::getPlannedPaths()
{
    return planned_paths;
}
