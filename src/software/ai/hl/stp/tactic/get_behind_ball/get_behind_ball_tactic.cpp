#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_tactic.h"

#include <algorithm>

GetBehindBallTactic::GetBehindBallTactic(
    std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<GetBehindBallFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

void GetBehindBallTactic::updateControlParams(const Point &ball_location,
                                              Angle chick_direction)
{
    control_params.ball_location   = ball_location;
    control_params.chick_direction = chick_direction;
}

void GetBehindBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
