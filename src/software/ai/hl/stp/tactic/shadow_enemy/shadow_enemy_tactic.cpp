#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShadowEnemyTactic::ShadowEnemyTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : Tactic<ShadowEnemyFSM>({RobotCapability::Move, RobotCapability::Kick}, ai_config_ptr)
{
}



void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

