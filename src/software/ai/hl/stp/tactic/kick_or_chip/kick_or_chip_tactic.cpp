#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_tactic.h"

#include <algorithm>


KickOrChipTactic::KickOrChipTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<KickOrChipFSM, GetBehindBallFSM>(
          {RobotCapability::Chip, RobotCapability::Move}, ai_config_ptr)
{
}


void KickOrChipTactic::updateControlParams(const Point& kick_or_chip_origin, const Angle& kick_or_chip_direction,
			     bool isChipping, double kick_speed_meters_per_second,
			     double chip_distance_meters);
{
    control_params.kick_or_chip_origin          = kick_or_chip_origin;
    control_params.kick_or_chip_direction       = kick_or_chip_direction;
    control_params.isChipping = isChipping;
    control_params.kick_speed_meters_per_second = kick_speed_meters_per_second;
    control_params.chip_distance_meters = chip_distance_meters;
}

void KickOrChipTactic::updateControlParams(const Point& kick_or_chip_origin, const Angle& kick_or_chip_target,
			     bool isChipping, double kick_speed_meters_per_second,
			     double chip_distance_meters);
{
    updateControlParams(kick_or_chip_origin, (kick_or_chip_target - kick_or_chip_origin).orientation(),
                        isChipping, kick_speed_meters_per_second, (kick_or_chip_target - kick_or_chip_origin).length());
}

void ChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
