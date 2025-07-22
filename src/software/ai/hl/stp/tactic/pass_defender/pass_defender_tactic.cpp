#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

PassDefenderTactic::PassDefenderTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<PassDefenderFSM>({RobotCapability::Move, RobotCapability::Kick}, ai_config_ptr),
      control_params(PassDefenderFSMControlParams())
{
}

std::unique_ptr<FSM<PassDefenderFSM>> PassDefenderTactic::fsm_init() {
    return std::make_unique<FSM<PassDefenderFSM>>(PassDefenderFSM(ai_config_ptr), DribbleFSM(ai_config_ptr));
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

void PassDefenderTactic::updatePrimitive(const TacticUpdate &tactic_update,
                                         bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(PassDefenderFSM::Update(control_params, tactic_update));
}
