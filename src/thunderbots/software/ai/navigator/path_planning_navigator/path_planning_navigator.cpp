#include "ai/navigator/path_planning_navigator/path_planning_navigator.h"

#include "ai/navigator/util.h"
#include "util/canvas_messenger/canvas_messenger.h"

std::vector<std::unique_ptr<Primitive>> PathPlanningNavigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world = world;

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
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

double PathPlanningNavigator::place_holder_violation_func(const Point &input_point)
{
    return 0.0;
}

void PathPlanningNavigator::visit(const MoveIntent &move_intent)
{
    std::vector<Obstacle> obstacles = {};

    for (auto &robot : world.enemyTeam().getAllRobots())
    {
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 0);
        obstacles.push_back(o);
    }

    for (auto &robot : world.friendlyTeam().getAllRobots())
    {
        if (robot.id() == move_intent.getRobotId())
        {
            // skip current robot
            continue;
        }
        Obstacle o = Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 0);
        obstacles.push_back(o);
    }

    // TODO: should we be using velocity scaling here?
    obstacles.push_back(Obstacle::createBallObstacle(world.ball(), 0.06, 0));

    auto p            = std::make_unique<MovePrimitive>(move_intent);
    auto path_planner = std::make_unique<ThetaStarPathPlanner>(
        this->world.field(), this->world.ball(), obstacles);

    auto path_points = path_planner->findPath(
        this->world.friendlyTeam().getRobotById(p->getRobotId())->position(),
        p->getDestination());

    if (path_points)
    {
        auto next_point = (*path_points)[1];
        auto move       = std::make_unique<MovePrimitive>(
            p->getRobotId(), next_point, move_intent.getFinalAngle(),
            calculateTransitionSpeedBetweenSegments((*path_points)[0], (*path_points)[1],
                                                    (*path_points)[2], 0),
            move_intent.isDribblerEnabled(), move_intent.isAutoKickEnabled());
        current_primitive = std::move(move);

        Util::CanvasMessenger::getInstance()->drawRobotPath(*path_points);
    }
    else
    {
        auto stop         = std::make_unique<StopPrimitive>(p->getRobotId(), false);
        current_primitive = std::move(stop);
    }
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
