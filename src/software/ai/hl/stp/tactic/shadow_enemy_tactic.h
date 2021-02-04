#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/tactic.h"

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
     * @param ball_steal_speed If the enemy robot has possession of the ball, and it is
     * moving less than this speed, the shadowing robot will try steal the ball
     * @param enemy_team_can_pass Whether or not we think the enemy team can pass the ball
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ShadowEnemyTactic(const Field &field, const Team &friendly_team,
                               const Team &enemy_team, bool ignore_goalie,
                               const Ball &ball, const double ball_steal_speed,
                               bool enemy_team_can_pass, bool loop_forever);

    ShadowEnemyTactic() = delete;

    void updateWorldParams(const World &world) override;

    /**
     * Updates the control parameters for this ShadowEnemyTactic
     *
     * @param enemy_threat The EnemyThreat indicating which enemy to shadow
     * @param shadow_distance How far from the enemy the robot will shadow. This is the
     * distance between the center of the enemy robot and the center of the robot
     * shadowing it
     */
    void updateControlParams(const EnemyThreat &enemy_threat, double shadow_distance);

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

    Ball getBall() const;
    Field getField() const;
    Team getFriendlyTeam() const;
    Team getEnemyTeam() const;

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;

    // Tactic parameters
    // The Enemy Threat indicating which enemy to shadow
    std::optional<EnemyThreat> enemy_threat;
    // The field being played on
    Field field;
    // The friendly team
    Team friendly_team;
    // The enemy team
    Team enemy_team;
    // Whether or not to ignore the friendly goalie when calculating the enemy's best shot
    // to shadow
    bool ignore_goalie;
    // The ball being played with
    Ball ball;
    // If the enemy robot has possession of the ball, and it is moving less than this
    // speed, the shadowing robot will try steal the ball
    double ball_steal_speed;
    // Whether or not we think the enemy team can pass the ball
    bool enemy_team_can_pass;
    // How far from the enemy the robot will position itself to shadow
    double shadow_distance;
};
