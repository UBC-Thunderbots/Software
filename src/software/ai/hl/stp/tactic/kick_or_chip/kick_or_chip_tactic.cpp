#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_tactic.h"

#include <algorithm>


KickOrChipTactic::KickOrChipTactic(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<KickOrChipFSM, GetBehindBallFSM>(
          {RobotCapability::Chip, RobotCapability::Move}, ai_config_ptr)
{
}


void KickOrChipTactic::updateControlParams(const Point& kick_or_chip_origin,
                                           const Angle& kick_or_chip_direction,
                                           AutoChipOrKick auto_chip_or_kick)
{
    control_params.kick_or_chip_origin    = kick_or_chip_origin;
    control_params.kick_or_chip_direction = kick_or_chip_direction;
    control_params.auto_chip_or_kick      = auto_chip_or_kick;
}

void KickOrChipTactic::updateControlParams(const Point& kick_or_chip_origin,
                                           const Point& kick_or_chip_target,
                                           AutoChipOrKick auto_chip_or_kick)
{
    updateControlParams(kick_or_chip_origin, kick_or_chip_target, auto_chip_or_kick);
}

void KickOrChipTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
