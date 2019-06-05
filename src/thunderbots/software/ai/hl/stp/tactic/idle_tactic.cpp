#include "ai/hl/stp/tactic/idle_tactic.h"

#include <algorithm>

#include "ai/hl/stp/action/idle_action.h"

IdleTactic::IdleTactic(bool loop_forever) : Tactic(loop_forever)
{
}

std::string IdleTactic::getName() const
{
    return "Idle Tactic";
}

void IdleTactic::updateParams()
{
    // The Idle Tactic has no parameters to update?
}

double IdleTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // 
    return;
}

void IdleTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    IdleAction idle_action = IdleAction();
    do
    {
        yield(idle_action.updateStateAndGetNextIntent(*robot);
    } while (!idle_action.done());
}
