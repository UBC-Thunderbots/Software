#include "ai/hl/stp/tactic/move.h"

MoveTactic::MoveTactic(const Point &destination) : destination(destination)
{
}

std::pair<Robot, Team> MoveTactic::selectRobot(const Team &available_robots)
{
    return std::make_pair(available_robots.getAllRobots().at(0), available_robots);
}

std::unique_ptr<Intent> MoveTactic::getNextIntent(const World &world, const Robot &robot)
{
    return std::make_unique<MoveIntent>(robot.id(), destination, Angle::zero(), 0.0);
}