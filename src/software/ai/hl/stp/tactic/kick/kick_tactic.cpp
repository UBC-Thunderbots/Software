#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

KickTactic::KickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
        : Tactic<KickFSM>({RobotCapability::Kick, RobotCapability::Move} ai_config_ptr)
{
}

std::unique_ptr<FSM<KickFSM>> KickTactic::fsm_init() {
    return std::make_unique<FSM<KickFSM>>(KickFSM(ai_config_ptr), GetBehindBallFSM(ai_config_ptr));
}

void KickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

