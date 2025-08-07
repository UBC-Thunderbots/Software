#pragma once


#include "../tactic_base.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class ShadowEnemyTactic : public TacticBase<ShadowEnemyFSM, MoveFSM>
{
   public:
    /**
     * Constructor for ShadowEnemyTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ShadowEnemyTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the control parameters for this ShadowEnemyTactic
     *
     * @param enemy_threat The EnemyThreat indicating which enemy to shadow
     * @param shadow_distance How far from the enemy the robot will shadow. This is the
     * distance between the center of the enemy robot and the center of the robot
     * shadowing it
     */
    void updateControlParams(std::optional<EnemyThreat> enemy_threat,
                             double shadow_distance);

    void accept(TacticVisitor &visitor) const override;
};
