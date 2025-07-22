#pragma once


#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class ShadowEnemyTactic : public Tactic<ShadowEnemyFSM>
{
   public:
    /**
     * Constructor for ShadowEnemyTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ShadowEnemyTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor &visitor) const override;
};
