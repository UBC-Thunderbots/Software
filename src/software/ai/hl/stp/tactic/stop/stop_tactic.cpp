#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"

#include <algorithm>

StopTactic::StopTactic()
    : Tactic(std::set<RobotCapability>()), fsm_map()
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<StopFSM>>(StopFSM());
    }
}

void StopTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void StopTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] =
            std::make_unique<FSM<StopFSM>>(StopFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(StopFSM::Update({}, tactic_update));
}
