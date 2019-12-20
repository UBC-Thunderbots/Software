#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

StopTactic::StopTactic(bool coast, bool loop_forever) : Tactic(loop_forever), coast(coast)
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

void StopTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(
        StopAction::ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT, false);
    do
    {
        stop_action->updateControlParams(*robot, this->coast);
        yield(stop_action);
    } while (!stop_action->done());
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
