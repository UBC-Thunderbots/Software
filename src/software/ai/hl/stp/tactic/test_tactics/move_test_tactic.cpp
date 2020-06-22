#include "software/ai/hl/stp/tactic/test_tactics/move_test_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/move_action.h"

MoveTestTactic::MoveTestTactic(bool loop_forever)
    : Tactic(loop_forever,
             {RobotCapabilities::Capability::Dribble, RobotCapabilities::Capability::Kick,
              RobotCapabilities::Capability::Chip})
{
}

std::string MoveTestTactic::getName() const
{
    return "Move Test Tactic";
}

void MoveTestTactic::updateControlParams(Point destination_)
{
    // Update the parameters stored by this Tactic
    this->destination = destination_;
}

double MoveTestTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with a constant factor so test results to not change based on any
    // changes to World
    double cost = (robot.position() - destination).length() / 10.0;
    return std::clamp<double>(cost, 0, 1);
}

void MoveTestTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    do
    {
        yield(std::make_shared<MoveAction>(false));
    } while ((this->robot->position() - this->destination).length() > 0.01);
}

void MoveTestTactic::accept(MutableTacticVisitor &visitor)
{
    visitor.visit(*this);
}
