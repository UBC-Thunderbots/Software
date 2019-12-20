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
        yield(std::make_shared<MoveAction>());
    } while ((this->robot->position() - this->destination).length() > 0.01);
}

void MoveTestTactic::accept(TacticVisitor &visitor) const
{
    // MoveTestTactic is meant to be a simple test tactic and so
    // we invoke YAGNI to not implement the visitor for this tactic
    throw std::invalid_argument(
        "Error: Tactic Visitor does not implement visiting this Tactic, so this accept function does nothing");
}
