#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<MoveFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
