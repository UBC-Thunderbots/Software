#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

StopTactic::StopTactic(bool coast, bool loop_forever) : coast(coast), Tactic(loop_forever)
{
}

std::string StopTactic::getName() const
{
    return "Stop Tactic";
}

double StopTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer all robots equally
    return 0.5;
}

void StopTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    StopAction stop_action =
        StopAction(StopAction::ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT, false);
    do
    {
        yield(stop_action.updateStateAndGetNextIntent(*robot, this->coast));
    } while (!stop_action.done());
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
