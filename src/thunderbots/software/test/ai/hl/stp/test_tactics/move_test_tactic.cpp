#include "test/ai/hl/stp/test_tactics/move_test_tactic.h"

#include <algorithm>

#include "ai/intent/move_intent.h"

MoveTestTactic::MoveTestTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapabilityFlags::Dribble, RobotCapabilityFlags::Kick,
                            RobotCapabilityFlags::Chip})
{
}

std::string MoveTestTactic::getName() const
{
    return "Move Test Tactic";
}

void MoveTestTactic::updateParams(Point destination_)
{
    // Update the parameters stored by this Tactic
    this->destination = destination_;
}

double MoveTestTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with a constant factor so test results to not change based on any
    // changes to World
    double cost = (robot.position() - destination).len() / 10.0;
    return std::clamp<double>(cost, 0, 1);
}

void MoveTestTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    do
    {
        yield(std::make_unique<MoveIntent>(this->robot->id(), this->destination,
                                           Angle::zero(), 0.0, 0));
    } while ((this->robot->position() - this->destination).len() > 0.01);
}
