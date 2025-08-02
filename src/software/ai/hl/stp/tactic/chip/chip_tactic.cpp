#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"

#include <algorithm>


ChipTactic::ChipTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ChipFSM, GetBehindBallFSM>({RobotCapability::Chip, RobotCapability::Move},
                                        ai_config_ptr)
{
}


void ChipTactic::updateControlParams(const Point &chip_origin,
                                     const Angle &chip_direction,
                                     double chip_distance_meters)
{
    control_params.chip_origin          = chip_origin;
    control_params.chip_direction       = chip_direction;
    control_params.chip_distance_meters = chip_distance_meters;
}

void ChipTactic::updateControlParams(const Point &chip_origin, const Point &chip_target)
{
    updateControlParams(chip_origin, (chip_target - chip_origin).orientation(),
                        (chip_target - chip_origin).length());
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
