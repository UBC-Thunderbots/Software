#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

#include <algorithm>

DribbleTactic::DribbleTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<DribbleFSM>({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}, ai_config_ptr),
      control_params{DribbleFSMControlParams{.dribble_destination       = std::nullopt,
                                               .final_dribble_orientation = std::nullopt,
                                               .allow_excessive_dribbling = false}},
{
}

void DribbleTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

