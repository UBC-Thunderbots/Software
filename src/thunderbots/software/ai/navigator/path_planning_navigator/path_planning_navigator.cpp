#include "ai/navigator/path_planning_navigator/path_planning_navigator.h"

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

#include <util/parameter/dynamic_parameters.h>

#include "ai/navigator/util.h"
#include "util/canvas_messenger/canvas_messenger.h"

// visitors
void PathPlanningNavigator::visit(const CatchIntent &catch_intent)
{
    auto p            = std::make_unique<CatchPrimitive>(catch_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const ChipIntent &chip_intent)
{
    auto p            = std::make_unique<ChipPrimitive>(chip_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const DirectVelocityIntent &direct_velocity_intent)
{
    auto p            = std::make_unique<DirectVelocityPrimitive>(direct_velocity_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const DirectWheelsIntent &direct_wheels_intent)
{
    auto p            = std::make_unique<DirectWheelsPrimitive>(direct_wheels_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const DribbleIntent &dribble_intent)
{
    auto p            = std::make_unique<DribblePrimitive>(dribble_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const KickIntent &kick_intent)
{
    auto p            = std::make_unique<KickPrimitive>(kick_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const MoveIntent &move_intent)
{
    Point start =
        this->world.friendlyTeam().getRobotById(move_intent.getRobotId())->position();
    Point dest = move_intent.getDestination();

    std::vector<Obstacle> obstacles =
        getCurrentObstacles(move_intent.getAreasToAvoid(), move_intent.getRobotId());

    auto path_planner =
        std::make_unique<ThetaStarPathPlanner>(this->world.field(), obstacles);

    auto path_points = path_planner->findPath(start, dest);

    if (path_points)
    {
        if ((*path_points).size() > 2)
        {
            current_destination = (*path_points)[1];
            auto move           = std::make_unique<MovePrimitive>(
                move_intent.getRobotId(), current_destination,
                move_intent.getFinalAngle(),
                calculateTransitionSpeedBetweenSegments(
                    (*path_points)[0], (*path_points)[1], (*path_points)[2], ROBOT_MAX_SPEED_METERS_PER_SECOND * Util::DynamicParameters::Navigator::transition_speed_factor.value()),
                move_intent.isDribblerEnabled(), move_intent.getAutoKickType());
            current_primitive = std::move(move);
            Util::CanvasMessenger::getInstance()->drawRobotPath(*path_points);
            return;
        }
        if ((*path_points).size() == 2)
        {
            current_destination = (*path_points)[1];
            auto move           = std::make_unique<MovePrimitive>(
                move_intent.getRobotId(), current_destination,
                move_intent.getFinalAngle(), 0, move_intent.isDribblerEnabled(),
                move_intent.getAutoKickType());
            current_primitive = std::move(move);
            Util::CanvasMessenger::getInstance()->drawRobotPath(*path_points);
            return;
        }
    }
    auto stop         = std::make_unique<StopPrimitive>(move_intent.getRobotId(), false);
    current_primitive = std::move(stop);
}

void PathPlanningNavigator::visit(const MoveSpinIntent &move_spin_intent)
{
    auto p            = std::make_unique<MoveSpinPrimitive>(move_spin_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const PivotIntent &pivot_intent)
{
    auto p            = std::make_unique<PivotPrimitive>(pivot_intent);
    current_primitive = std::move(p);
}

void PathPlanningNavigator::visit(const StopIntent &stop_intent)
{
    auto p            = std::make_unique<StopPrimitive>(stop_intent);
    current_primitive = std::move(p);
}

// helpers
std::optional<Obstacle> PathPlanningNavigator::obstacleFromAvoidArea(AvoidArea avoid_area)
{
    Rectangle rectangle({0, 0}, {0, 0});
    switch (avoid_area)
    {
        case AvoidArea::FRIENDLY_DEFENSE_AREA:
            // We extend the friendly defense area back by several meters to prevent
            // robots going around the back of the goal
            rectangle =
                Rectangle(world.field().friendlyDefenseArea().neCorner(),
                          Point(-10, world.field().friendlyDefenseArea().seCorner().y()));
            rectangle.expand(OBSTACLE_INFLATION_DIST);
            return Obstacle(rectangle);
        case AvoidArea::ENEMY_DEFENSE_AREA:
            // We extend the enemy defense area back by several meters to prevent
            // robots going around the back of the goal
            rectangle =
                Rectangle(world.field().enemyDefenseArea().nwCorner(),
                          Point(10, world.field().enemyDefenseArea().swCorner().y()));
            rectangle.expand(OBSTACLE_INFLATION_DIST);
            return Obstacle(rectangle);
        case AvoidArea::INFLATED_ENEMY_DEFENSE_AREA:
            rectangle = world.field().enemyDefenseArea();
            rectangle.expand(OBSTACLE_INFLATION_DIST + 0.2);
            return Obstacle(rectangle);
        case AvoidArea::CENTER_CIRCLE:
            return Obstacle::createCircleObstacle(
                world.field().centerPoint(), world.field().centreCircleRadius(), 1.2);
        case AvoidArea::HALF_METER_AROUND_BALL:
            return Obstacle::createCircleObstacle(world.ball().position(), 0.5, 1.2);
        case AvoidArea::BALL:
            return Obstacle::createCircularBallObstacle(world.ball(), 0.06);
        case AvoidArea::ENEMY_HALF:
            rectangle =
                Rectangle({0, world.field().width() / 2}, world.field().enemyCornerNeg());
            rectangle.expand(OBSTACLE_INFLATION_DIST);
            return Obstacle(rectangle);
        case AvoidArea::FRIENDLY_HALF:
            rectangle = Rectangle({0, world.field().width() / 2},
                                  world.field().friendlyCornerNeg());
            rectangle.expand(OBSTACLE_INFLATION_DIST);
            return Obstacle(rectangle);
        default:
            LOG(WARNING) << "Could not convert AvoidArea " << (int)avoid_area
                         << " to obstacle";
    }

    return std::nullopt;
}

std::vector<std::unique_ptr<Primitive>> PathPlanningNavigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world              = world;
    this->current_robot      = std::nullopt;
    this->velocity_obstacles = {};

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (this->current_robot)
        {
            this->velocity_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    this->current_robot->position(), this->current_destination,
                    this->current_robot->velocity().len(), 1.2, .04));
            this->current_robot = std::nullopt;
        }
        assigned_primitives.emplace_back(std::move(current_primitive));
    }

    Util::CanvasMessenger::getInstance()->publishAndClearLayer(
        Util::CanvasMessenger::Layer::NAVIGATOR);

    return assigned_primitives;
}

std::vector<Obstacle> PathPlanningNavigator::getCurrentObstacles(
    const std::vector<AvoidArea> &avoid_areas, int robot_id)
{
    std::vector<Obstacle> obstacles;

    // Avoid obstacles specific to this MoveIntent
    for (auto area : avoid_areas)
    {
        auto obstacle_opt = obstacleFromAvoidArea(area);
        if (obstacle_opt)
        {
            obstacles.emplace_back(*obstacle_opt);
            // draw the avoid area
            drawObstacle(*obstacle_opt, Util::CanvasMessenger::AVOID_AREA_COLOR);
        }
    }

    for (auto &robot : world.enemyTeam().getAllRobots())
    {
        //@todo consider using velocity obstacles: Obstacle o =
        // Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 0);
        Obstacle o =
            Obstacle::createCircularRobotObstacle(robot, ROBOT_OBSTACLE_INFLATION_FACTOR);
        obstacles.push_back(o);
        drawObstacle(o, Util::CanvasMessenger::ENEMY_TEAM_COLOR);
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
        Obstacle o =
            Obstacle::createCircularRobotObstacle(robot, ROBOT_OBSTACLE_INFLATION_FACTOR);
        obstacles.push_back(o);
        drawObstacle(o, Util::CanvasMessenger::FRIENDLY_TEAM_COLOR);
    }

    return obstacles;
}

void PathPlanningNavigator::drawObstacle(const Obstacle &obstacle,
                                         const Util::CanvasMessenger::Color &color)
{
    if (obstacle.getBoundaryPolygon())
    {
        Util::CanvasMessenger::getInstance()->drawPolygonOutline(
            Util::CanvasMessenger::Layer::NAVIGATOR, *obstacle.getBoundaryPolygon(),
            0.025, color);
    }
    else if (obstacle.getBoundaryCircle())
    {
        Util::CanvasMessenger::getInstance()->drawPolygonOutline(
            Util::CanvasMessenger::Layer::NAVIGATOR,
            circleToPolygon(*obstacle.getBoundaryCircle(), 12), 0.025, color);
    }
}
