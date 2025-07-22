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
    : Tactic<CreaseDefenderFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

std::unique_ptr<FSM<CreaseDefenderFSM>> CreaseDefenderTactic::fsm_init() {
    return std::make_unique<FSM<CreaseDefenderFSM>>(CreaseDefenderFSM(ai_config_ptr), DribbleFSM(ai_config_ptr));
}

void CreaseDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}


