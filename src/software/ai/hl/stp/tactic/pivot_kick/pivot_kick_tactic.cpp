#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

PivotKickTactic::PivotKickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<PivotKickFSM, DribbleFSM>({RobotCapability::Move, RobotCapability::Kick,
                                        RobotCapability::Chip, RobotCapability::Dribble},
                                       ai_config_ptr)
{
}

void PivotKickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PivotKickTactic::updateControlParams(const Point &kick_origin,
                                          const Angle &kick_direction,
                                          AutoChipOrKick auto_chip_or_kick)
{
    control_params.kick_origin       = kick_origin;
    control_params.kick_direction    = kick_direction;
    control_params.auto_chip_or_kick = auto_chip_or_kick;
}
