#include "ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

#include "ai/hl/stp/action/stop_action.h"

StopTactic::StopTactic(bool coast, bool loop_forever) : coast(coast), Tactic(loop_forever)
{
}

std::string StopTactic::getName() const
{
    return "Stop Tactic";
}

void StopTactic::updateParams()
{
    // The Stop Tactic has no parameters to update
}

double StopTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer all robots equally
    return 0.5;
}

void StopTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    StopAction stop_action = StopAction(StopAction::ROBOT_STOPPED_SPEED_THRESHOLD, false);
    do
    {
        yield(stop_action.updateStateAndGetNextIntent(*robot, this->coast));
    } while (!stop_action.done());
}
