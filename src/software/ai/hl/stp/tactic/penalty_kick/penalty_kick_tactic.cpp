#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<PenaltyKickFSM>({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}, ai_config_ptr)
{
}

std::unique_ptr<FSM<PenaltyKickFSM>> PenaltyKickTactic::fsm_init() {
    return std::make_unique<FSM<PenaltyKickFSM>>(
            DribbleFSM(ai_config_ptr),
            PenaltyKickFSM(ai_config_ptr),
            GetBehindBallFSM(ai_config_ptr));
}

void PenaltyKickTactic::updateControlParams() {}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}