#pragma once

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class ShadowEnemyTactic : public Tactic
{
   public:
    /**
     * Creates a new ShadowEnemyTactic
     *
     * @param field The field being played on
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param ignore_goalie Whether or not to ignore the friendly goalie when calculating
     * where to shadow the enemy
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ShadowEnemyTactic(const Field &field, const Team &friendly_team,
                               const Team &enemy_team, bool ignore_goalie,
                               bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this ShadowEnemyTactic.
     *
     * @param enemy_threat The EnemyThreat indicating which enemy to shadow
     * @param field The field being played on
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param enemy_team_can_pass Whether or not we think the enemy team can pass the ball
     * @param shadow_distance How far from the enemy the robot will shadow. This is the
     * distance between the center of the enemy robot and the center of the robot
     * shadowing it
     * @param enemy_team_can_pass Whether or not the enemy team can pass. If this value is
     * true, the robot will try block passes to the robot being shadowed. Otherwise, it
     * will block the net.
     */
    void updateParams(const Evaluation::EnemyThreat &enemy_threat, const Field &field,
                      const Team &friendly_team, const Team &enemy_team,
                      double shadow_distance, bool enemy_team_can_pass);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the enemy being shadowed
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    // Tactic parameters
    // The Enemy Threat indicating which enemy to shadow
    std::optional<Evaluation::EnemyThreat> enemy_threat;
    // The field being played on
    Field field;
    // The friendly team
    Team friendly_team;
    // The enemy team
    Team enemy_team;
    // How far from the enemy the robot will position itself to shadow
    double shadow_distance;
    // Whether or not we think the enemy team can pass the ball
    bool enemy_team_can_pass;
    // Whether or not to ignore the friendly goalie when calculating the enemy's best shot
    // to shadow
    bool ignore_goalie;
};
