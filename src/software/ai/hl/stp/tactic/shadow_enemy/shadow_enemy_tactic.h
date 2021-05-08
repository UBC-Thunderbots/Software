#pragma once


#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class ShadowEnemyTactic : public Tactic
{
   public:
    explicit ShadowEnemyTactic();

    void updateWorldParams(const World &world) override;

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

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the enemy being shadowed
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;



   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    FSM<ShadowEnemyFSM> fsm;
    ShadowEnemyFSM::ControlParams control_params;
};
