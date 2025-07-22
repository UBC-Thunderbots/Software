#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<GetBehindBallFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

void GetBehindBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
