#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"

#include <algorithm>

HaltTactic::HaltTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<HaltFSM>(std::set<RobotCapability>(), ai_config_ptr)
{
}

void HaltTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
