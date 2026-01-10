#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

#include <algorithm>

DribbleTactic::DribbleTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<DribbleFSM>(
          {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick},
          ai_config_ptr)
{
}

void DribbleTactic::updateControlParams(
    std::optional<Point> dribble_destination,
    std::optional<Angle> final_dribble_orientation, bool allow_excessive_dribbling,
    TbotsProto::MaxAllowedSpeedMode max_speed_dribble,
    TbotsProto::MaxAllowedSpeedMode max_speed_get_possession)
{
    control_params.dribble_destination       = dribble_destination;
    control_params.final_dribble_orientation = final_dribble_orientation;
    control_params.allow_excessive_dribbling = allow_excessive_dribbling;
    control_params.max_speed_dribble         = max_speed_dribble;
    control_params.max_speed_get_possession  = max_speed_get_possession;
}

void DribbleTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
