#pragma once

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class DefenseShadowEnemyTactic : public Tactic
{
   public:

    explicit DefenseShadowEnemyTactic(const Field &field, const Team &friendly_team,
                               const Team &enemy_team, const Ball& ball, bool ignore_goalie,
                               bool loop_forever = false);

    std::string getName() const override;

    void updateParams(const Evaluation::EnemyThreat &enemy_threat, const Field &field,
                      const Team &friendly_team, const Team &enemy_team, double shadow_distance,
                      const Ball& ball, bool enemy_team_can_pass);

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
    Ball ball;
    // How far from the enemy the robot will position itself to shadow
    double shadow_distance;
    // Whether or not we think the enemy team can pass the ball
    bool enemy_team_can_pass;
    // Whether or not to ignore the friendly goalie when calculating the enemy's best shot
    // to shadow
    bool ignore_goalie;
};
