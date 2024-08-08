#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShadowEnemyTactic::ShadowEnemyTactic(std::shared_ptr<Strategy> strategy)
    : Tactic({RobotCapability::Move, RobotCapability::Kick}),
      strategy(strategy),
      fsm_map(),
      control_params{ShadowEnemyFSM::ControlParams{.enemy_threat    = std::nullopt,
                                                   .shadow_distance = 0}}
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<ShadowEnemyFSM>>(ShadowEnemyFSM(strategy),
                                                            MoveSkillFSM());
    }
}

void ShadowEnemyTactic::updateControlParams(std::optional<EnemyThreat> enemy_threat,
                                            double shadow_distance)
{
    control_params.enemy_threat    = enemy_threat;
    control_params.shadow_distance = shadow_distance;
}

void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void ShadowEnemyTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<ShadowEnemyFSM>>(
            ShadowEnemyFSM(strategy), MoveSkillFSM());
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}
