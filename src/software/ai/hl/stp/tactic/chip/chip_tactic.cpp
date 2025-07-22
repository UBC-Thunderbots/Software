#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"

#include <algorithm>


ChipTactic::ChipTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ChipFSM>({RobotCapability::Chip, RobotCapability::Move}, ai_config_ptr)
{
}

std::unique_ptr<FSM<ChipFSM>> ChipTactic::fsm_init() {
    return std::make_unique<FSM<ChipFSM>>(GetBehindBallFSM(ai_config_ptr), ChipFSM(ai_config_ptr));
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

