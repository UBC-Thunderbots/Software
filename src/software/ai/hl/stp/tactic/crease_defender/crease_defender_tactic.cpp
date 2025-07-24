#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/logger/logger.h"

CreaseDefenderTactic::CreaseDefenderTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<CreaseDefenderFSM, DribbleFSM, MoveFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

void CreaseDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void CreaseDefenderTactic::updateControlParams(
    const Point &enemy_threat_origin,
    const TbotsProto::CreaseDefenderAlignment &alignment,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    TbotsProto::BallStealMode ball_steal_mode)
{
    control_params.enemy_threat_origin       = enemy_threat_origin;
    control_params.crease_defender_alignment = alignment;
    control_params.max_allowed_speed_mode    = max_allowed_speed_mode;
    control_params.ball_steal_mode           = ball_steal_mode;
}

