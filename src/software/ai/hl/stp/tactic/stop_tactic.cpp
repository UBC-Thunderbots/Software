#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

#include "software/ai/intent/stop_intent.h"

StopTactic::StopTactic(bool coast) : Tactic(true, {}), coast(coast) {}

double StopTactic::cost(const Robot &robot, const World &world) const
{
    // Prefer all robots equally
    return 0.5;
}

bool StopTactic::done() const
{
    return false;
}

void StopTactic::updateFSM(const Robot &robot, const World &world)
{
    StopTacticUpdate event{.robot = robot, .world = world};
    // fsm.process_event(event);
    // ^this modifies intent
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
