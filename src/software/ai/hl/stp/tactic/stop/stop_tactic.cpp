#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"

#include <algorithm>

StopTactic::StopTactic(bool coast) : Tactic(std::set<RobotCapability>()), fsm(StopFSM(coast)) {}

double StopTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Prefer all robots equally
    return 0.5;
}

bool StopTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void StopTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(StopFSM::Update({}, tactic_update));
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
