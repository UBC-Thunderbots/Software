#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <algorithm>

StopTactic::StopTactic(bool coast) : Tactic(true, {}), coast(coast) {}

double StopTactic::cost(const Robot &robot, const World &world) const
{
    // Prefer all robots equally
    return 0.5;
}

bool StopTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void StopTactic::updateFSM(const Robot &robot, const World &world)
{
    StopTacticFSM::Update event{
        .coast  = coast,
        .common = {.robot      = robot,
                   .world      = world,
                   .set_intent = [this](std::unique_ptr<Intent> new_intent) {
                       intent = std::move(new_intent);
                   }}};
    fsm.process_event(event);
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
