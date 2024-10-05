#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"

#include <algorithm>

HaltTactic::HaltTactic() : Tactic(std::set<RobotCapability>()), fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<HaltFSM>>(HaltFSM());
    }
}

void HaltTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void HaltTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<HaltFSM>>(HaltFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(HaltFSM::Update({}, tactic_update));
}
