#include "software/ai/hl/stp/tactic/test_tactics/stop_test_tactic.h"

#include "software/ai/hl/stp/action/stop_action.h"

StopTestTactic::StopTestTactic(bool loop_forever) : Tactic(loop_forever, {}) {}

double StopTestTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Prefer all robots equally with a cost of 0.5
    return 0.5;
}

void StopTestTactic::updateWorldParams(const World &world) {}

void StopTestTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    do
    {
        yield(std::make_shared<StopAction>(false));
    } while (this->robot_->velocity().length() > 0.05);
}

void StopTestTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
