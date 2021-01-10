#include "software/ai/hl/stp/tactic/test_tactics/move_test_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/move_action.h"

MoveTestTactic::MoveTestTactic(bool loop_forever)
    : Tactic(loop_forever, allRobotCapabilities())
{
}

void MoveTestTactic::updateWorldParams(const World &world) {}

void MoveTestTactic::updateControlParams(Point destination_)
{
    // Update the parameters stored by this Tactic
    this->destination = destination_;
}

double MoveTestTactic::calculateRobotCost(const Robot &robot, const World &world) const
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
    } while ((this->robot_->position() - this->destination).length() > 0.01);
}

void MoveTestTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
