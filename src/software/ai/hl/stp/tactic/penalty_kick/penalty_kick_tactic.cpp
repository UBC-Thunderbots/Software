#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_tactic.h"

PenaltyKickTactic::PenaltyKickTactic(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<PenaltyKickFSM, DribbleFSM, KickFSM, GetBehindBallFSM>(
          {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick},
          ai_config_ptr)
{
}

void PenaltyKickTactic::updateControlParams() {}

void PenaltyKickTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
