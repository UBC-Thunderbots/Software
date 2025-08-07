#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"

#include <algorithm>

KickTactic::KickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<KickFSM, GetBehindBallFSM>({RobotCapability::Kick, RobotCapability::Move},
                                            ai_config_ptr)
{
}

void KickTactic::updateControlParams(const Point &kick_origin,
                                     const Angle &kick_direction,
                                     double kick_speed_meters_per_second)
{
    control_params.kick_origin                  = kick_origin;
    control_params.kick_direction               = kick_direction;
    control_params.kick_speed_meters_per_second = kick_speed_meters_per_second;
}

void KickTactic::updateControlParams(const Point &kick_origin, const Point &kick_target,
                                     double kick_speed_meters_per_second)
{
    updateControlParams(kick_origin, (kick_target - kick_origin).orientation(),
                        kick_speed_meters_per_second);
}

void KickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
