#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

#include <algorithm>

DribbleTactic::DribbleTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<DribbleFSM>(
          {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick},
          ai_config_ptr)
{
}

void DribbleTactic::updateControlParams(std::optional<Point> dribble_destination,
                                        std::optional<Angle> final_dribble_orientation,
                                        bool allow_excessive_dribbling)
{
    control_params.dribble_destination       = dribble_destination;
    control_params.final_dribble_orientation = final_dribble_orientation;
    control_params.allow_excessive_dribbling = allow_excessive_dribbling;
}

void DribbleTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
