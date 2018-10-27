#include "ai/hl/stp/tactic/move.h"

MoveTactic::MoveTactic(const Point &destination) : destination(destination) {}

Robot MoveTactic::selectRobot(const World &world,
                              const std::vector<Robot> &available_robots)
{
    // Placeholder for now
    return Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                 std::chrono::steady_clock::now());
}

std::unique_ptr<Intent> MoveTactic::getNextIntent(const World &world, const Robot &robot)
{
    // Placeholder for now
    return std::make_unique<MoveIntent>(robot.id(), destination, Angle::zero(), 0.0);
}