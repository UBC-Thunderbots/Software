#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency

StopTactic::StopTactic(bool coast) : Tactic(true, {}), coast(coast) {}

double StopTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Prefer all robots equally
    return 0.5;
}

void StopTactic::updateWorldParams(const World &world) {}

void StopTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, this->coast);
        yield(stop_action);
    } while (!stop_action->done());
}

bool StopTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void StopTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(StopFSM::Update(coast, tactic_update));
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
