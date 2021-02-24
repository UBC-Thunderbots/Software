#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/parameter/dynamic_parameters.h"

/**
 * The ShootGoalTactic will make the assigned robot shoot on the enemy net
 */
class ShootGoalTactic : public Tactic
{
   public:
    /**
     * Creates a new ShootGoalTactic
     *
     * @param field The field being played on
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param ball The ball
     * @param min_net_open_angle The minimum open angle we must be able to see of the
     * goal in order to shoot
     * @param chip_target An optional point that the robot will chip towards when it is
     * unable to shoot and is in danger of losing the ball to an enemy. If this value is
     * not provided, the point defaults to the enemy goal
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     * @param shoot_goal_tactic_config The config to fetch parameters from
     */
    explicit ShootGoalTactic(
        const Field& field, const Team& friendly_team, const Team& enemy_team,
        const Ball& ball, Angle min_net_open_angle, std::optional<Point> chip_target,
        bool loop_forever,
        std::shared_ptr<const ShootGoalTacticConfig> shoot_goal_tactic_config);

    ShootGoalTactic() = delete;

    void updateWorldParams(const World& world) override;


    /**
     * Updates the control parameters for this ShootGoalTactic
     *
     * @param chip_target An optional point that the robot will chip towards when it is
     * unable to shoot and is in danger of losing the ball to an enemy. If this value is
     * not provided, the point defaults to the enemy goal
     */
    void updateControlParams(std::optional<Point> chip_target);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the ball
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    /**
     * Returns true if the robot has found a shot with at least the minimum percentage of
     * the net open
     *
     * @return true if the robot has found a shot with at least the minimum percentage of
     * the net open, and false otherwise
     */
    bool hasShotAvailable() const;

    void accept(TacticVisitor& visitor) const override;

    Ball getBall() const;
    Field getField() const;
    Team getFriendlyTeam() const;
    Team getEnemyTeam() const;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;

    /**
     * Returns true if there is an enemy close enough to steal the ball from the shooting
     * robot
     *
     * @return true if there is an enemy close enough to steal the ball from the shooting
     * robot
     */
    bool isEnemyAboutToStealBall() const;

    /**
     * A helper function that will continuously yield kick or chip intents until there is
     * no possible shot on net
     *
     * @param kick_action The kick action to run
     * @param chip_action The chip action to run
     * @param yield The coroutine to yield
     */
    void shootUntilShotBlocked(std::shared_ptr<KickAction> kick_action,
                               std::shared_ptr<ChipAction> chip_action,
                               ActionCoroutine::push_type& yield) const;

    // Tactic parameters
    // The field being played on
    Field field;
    // The friendly team
    Team friendly_team;
    // The enemy team
    Team enemy_team;
    // The ball
    Ball ball;
    // The minimum open angle we must be able to see of the goal in order to shoot
    Angle min_net_open_angle;
    // The point the robot will chip towards if it is unable to shoot and is in danger or
    // losing the ball to an enemy
    std::optional<Point> chip_target;
    // Whether or not there is currently a shot available with at least the minimum
    // percentage of the net open
    bool has_shot_available;

    // The distance between the robot's kicker and the ball while tracking the ball
    // waiting for a shot
    const double TRACK_BALL_DIST = 0.05;

    std::shared_ptr<const ShootGoalTacticConfig> shoot_goal_tactic_config;
};
