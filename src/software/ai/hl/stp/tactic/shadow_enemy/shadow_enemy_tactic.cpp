#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShadowEnemyTactic::ShadowEnemyTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ShadowEnemyFSM>({RobotCapability::Move, RobotCapability::Kick}, ai_config_ptr),
      control_params{ShadowEnemyFSMControlParams{.enemy_threat    = std::nullopt,
                                                   .shadow_distance = 0}}
{
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
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}
