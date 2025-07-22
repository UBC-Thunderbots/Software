#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/logger/logger.h"

ReceiverTactic::ReceiverTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ReceiverFSM>({RobotCapability::Move}, ai_config_ptr)
{
}


void ReceiverTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}

