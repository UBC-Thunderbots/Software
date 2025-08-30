#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShadowEnemyTactic::ShadowEnemyTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<ShadowEnemyFSM, MoveFSM>({RobotCapability::Move, RobotCapability::Kick},
                                          ai_config_ptr)
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
