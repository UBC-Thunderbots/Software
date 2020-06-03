#include "software/ai/hl/stp/tactic/test_tactics/goalie_test_tactic.h"

GoalieTestTactic::GoalieTestTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string GoalieTestTactic::getName() const
{
    return "Goalie Test Tactic";
}

bool GoalieTestTactic::isGoalieTactic() const
{
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

void GoalieTestTactic::accept(MutableTacticVisitor &visitor)
{
    visitor.visit(*this);
}
