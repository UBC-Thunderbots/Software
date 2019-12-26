#include "software/ai/hl/stp/tactic/test_tactics/goalie_test_tactic.h"

GoalieTestTactic::GoalieTestTactic(bool loop_forever) : Tactic(loop_forever) {
}

std::string GoalieTestTactic::getName() const
{
    return "Goalie Test Tactic";
}

bool GoalieTestTactic::isGoalieTactic() {
    return true;
}

double GoalieTestTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer all robots equally with a cost of 0.5
    return 0.5;
}

void GoalieTestTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    // Yield nothing
}

void GoalieTestTactic::accept(TacticVisitor &visitor) const
{
    // GoalieTestTactic is meant to be a simple test tactic and so
    // we invoke YAGNI to not implement the visitor for this tactic
    throw std::invalid_argument(
        "Error: Tactic Visitor does not implement visiting this Tactic, so this accept function does nothing");
}
