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
    : Tactic<PivotKickFSM>({RobotCapability::Move, RobotCapability::Kick, RobotCapability::Chip,
              RobotCapability::Dribble}, ai_config_ptr)
{
}

std::unique_ptr<FSM<PivotKickFSM>> PivotKickTactic::fsm_init() {
    return std::make_unique<FSM<PivotKickFSM>>(
            PivotKickFSM(ai_config_ptr),
            DribbleFSM(ai_config_ptr));
}

void PivotKickTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

