#include "software/ai/hl/stp/tactic/pass_defender/pass_defender_tactic.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"

PassDefenderTactic::PassDefenderTactic(TbotsProto::AiConfig ai_config)
    : Tactic({RobotCapability::Move, RobotCapability::Kick}),
      fsm_map(),
      control_params(PassDefenderFSM::ControlParams()),
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<PassDefenderFSM>>(
                PassDefenderFSM(),
                DribbleFSM(ai_config.dribble_tactic_config()));
    }
}

void PassDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void PassDefenderTactic::updateControlParams(
        const Point &position_to_block_from,
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
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<PassDefenderFSM>>(
                PassDefenderFSM(),
                DribbleFSM(ai_config.dribble_tactic_config()));
    }
    Point ball_position       = tactic_update.world_ptr->ball().position();
    Point enemy_goal_center   = tactic_update.world_ptr->field().enemyGoal().centre();
    Vector ball_to_net_vector = Vector(enemy_goal_center.x() - ball_position.x(),
                                       enemy_goal_center.y() - ball_position.y());
    DribbleFSM::ControlParams dribble_control_params{
            .dribble_destination       = ball_position,
            .final_dribble_orientation = ball_to_net_vector.orientation(),
            .allow_excessive_dribbling = false};


    fsm_map.at(tactic_update.robot.id())
        ->process_event(PassDefenderFSM::Update(control_params, tactic_update));
    fsm_map.at(tactic_update.robot.id())
            ->process_event(DribbleFSM::Update(dribble_control_params, tactic_update));
}
