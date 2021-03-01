#pragma once

#include "shared/parameter_v2/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowEnemyTactic will shadow and mark the robot specified in the given
 * EnemyThreat. It will choose to either block the enemy's shot on net or the pass it
 * would receive from another enemy.
 */
class DefenseShadowEnemyTactic : public Tactic
{
   public:
    /**
     * Creates a new DefenseShadowEnemyTactic
     *
     * @param field The field
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param ball The ball
     * @param ignore_goalie Whether or not to ignore the friendly goalie when determining
     * the best position to shadow in order to block shots
     * @param shadow_distance How far away to shadow the enemy from
     * @param defense_shadow_enemy_tactic_config The config to fetch parameters from
     */
    explicit DefenseShadowEnemyTactic(
        const Field &field, const Team &friendly_team, const Team &enemy_team,
        const Ball &ball, bool ignore_goalie, double shadow_distance,
        std::shared_ptr<const DefenseShadowEnemyTacticConfig>
            defense_shadow_enemy_tactic_config);

    DefenseShadowEnemyTactic() = delete;

    void updateWorldParams(const World &world) override;

    /**
     * Updates the control parameters for this Tactic
     *
     * @param enemy_threat The EnemyThread this Tactic is shadowing
     */
    void updateControlParams(const EnemyThreat &enemy_threat);

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
    Ball ball;
    // Whether or not to ignore the friendly goalie when calculating the enemy's best shot
    // to shadow
    bool ignore_goalie;
    // How far from the enemy the robot will position itself to shadow
    double shadow_distance;
    std::shared_ptr<const DefenseShadowEnemyTacticConfig>
        defense_shadow_enemy_tactic_config;
};
