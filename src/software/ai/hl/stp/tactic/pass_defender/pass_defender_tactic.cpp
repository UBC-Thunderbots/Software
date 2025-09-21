#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

PassDefenderTactic::PassDefenderTactic(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<PassDefenderFSM, DribbleFSM>(
          {RobotCapability::Move, RobotCapability::Kick}, ai_config_ptr)
{
}

void PassDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PassDefenderTactic::updateControlParams(const Point &position_to_block_from,
                                             TbotsProto::BallStealMode ball_steal_mode)
{
    control_params.position_to_block_from = position_to_block_from;
    control_params.ball_steal_mode        = ball_steal_mode;
}
