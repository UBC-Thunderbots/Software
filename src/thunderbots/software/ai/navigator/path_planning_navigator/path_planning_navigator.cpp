#include "ai/navigator/path_planning_navigator/path_planning_navigator.h"

#include "ai/navigator/util.h"
#include "util/canvas_messenger/canvas_messenger.h"

std::vector<std::unique_ptr<Primitive>> PathPlanningNavigator::getAssignedPrimitives(
    const World &world, const std::vector<Obstacle> &additional_obstacles,
    const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world                = world;
    this->current_robot        = std::nullopt;
    this->additional_obstacles = additional_obstacles;

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (this->current_robot)
        {
            this->additional_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    this->current_robot->position(), this->current_destination,
                    this->current_robot->velocity().len(), 1.2, .04));
            this->current_robot = std::nullopt;
        }
        assigned_primitives.emplace_back(std::move(current_primitive));
    }

    return assigned_primitives;
}

std::vector<std::unique_ptr<Primitive>> PathPlanningNavigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world                = world;
    this->current_robot        = std::nullopt;
    this->additional_obstacles = {};

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        if (this->current_robot)
        {
            this->additional_obstacles.emplace_back(
                Obstacle::createVelocityObstacleWithScalingParams(
                    this->current_robot->position(), this->current_destination,
                    this->current_robot->velocity().len(), 1.2, .04));
            this->current_robot = std::nullopt;
        }
        assigned_primitives.emplace_back(std::move(current_primitive));
    }

    return assigned_primitives;
}

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
    auto p      = std::make_unique<MovePrimitive>(move_intent);
    Point start = this->world.friendlyTeam().getRobotById(p->getRobotId())->position();
    Point dest  = p->getDestination();

    std::vector<Obstacle> obstacles = this->additional_obstacles;  // copy assignment

    for (auto &robot : world.enemyTeam().getAllRobots())
    {
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 0);
        obstacles.push_back(o);
    }

    for (auto &robot : world.friendlyTeam().getAllRobots())
    {
        if (robot.id() == move_intent.getRobotId())
        {
            // store current robot
            this->current_robot = robot;
            // skip current robot
            continue;
        }
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 0);
        obstacles.push_back(o);
    }

    for (auto o : move_intent.getAdditionalObstacles())
    {
        obstacles.push_back(o);
    }

    // TODO: should we be using velocity scaling here?
    obstacles.push_back(Obstacle::createBallObstacle(world.ball(), 0.06, 0));

    auto path_planner =
        std::make_unique<ThetaStarPathPlanner>(this->world.field(), obstacles);

    auto path_points = path_planner->findPath(start, dest);

    if (path_points)
    {
        if ((*path_points).size() > 2)
        {
            current_destination = (*path_points)[1];
            auto move           = std::make_unique<MovePrimitive>(
                p->getRobotId(), current_destination, move_intent.getFinalAngle(),
                calculateTransitionSpeedBetweenSegments(
                    (*path_points)[0], (*path_points)[1], (*path_points)[2], 0),
                move_intent.isDribblerEnabled(), move_intent.getAutoKickType());
            current_primitive = std::move(move);
            Util::CanvasMessenger::getInstance()->drawRobotPath(*path_points);
            return;
        }
        if ((*path_points).size() == 2)
        {
            current_destination = (*path_points)[1];
            auto move           = std::make_unique<MovePrimitive>(
                p->getRobotId(), current_destination, move_intent.getFinalAngle(), 0,
                move_intent.isDribblerEnabled(), move_intent.getAutoKickType());
            current_primitive = std::move(move);
            Util::CanvasMessenger::getInstance()->drawRobotPath(*path_points);
            return;
        }
    }
    auto stop         = std::make_unique<StopPrimitive>(p->getRobotId(), false);
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
