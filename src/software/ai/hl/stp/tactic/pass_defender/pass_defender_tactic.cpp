#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

PassDefenderTactic::PassDefenderTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<PassDefenderFSM>({RobotCapability::Move, RobotCapability::Kick}, ai_config_ptr)
{
}

std::unique_ptr<FSM<PassDefenderFSM>> PassDefenderTactic::fsm_init() {
    return std::make_unique<FSM<PassDefenderFSM>>(PassDefenderFSM(ai_config_ptr), DribbleFSM(ai_config_ptr));
}

void PassDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

