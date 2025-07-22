#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"

#include <algorithm>

HaltTactic::HaltTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr) : Tactic<HaltFSM>(std::set<RobotCapability>(), ai_config_ptr)
{
}

void HaltTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void HaltTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(HaltFSM::Update({}, tactic_update));
}
