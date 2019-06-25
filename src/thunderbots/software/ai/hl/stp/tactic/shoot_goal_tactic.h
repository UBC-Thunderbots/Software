#pragma once

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/tactic/tactic.h"
#include "shared/constants.h"

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
     * @param min_percent_net_open A value in the range [0, 1.0] that indicates the
     * fraction of the net that must be open in order for the robot to take a shot
     * @param chip_target An optional point that the robot will chip towards when it is
     * unable to shoot and is in danger of losing the ball to an enemy. If this value is
     * not provided, the point defaults to the enemy goal
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ShootGoalTactic(const Field& field, const Team& friendly_team,
                             const Team& enemy_team, const Ball& ball,
                             double min_percent_net_open,
                             std::optional<Point> chip_target, bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this ShootGoalTactic
     *
     * @param field The field being played on
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param ball The ball
     */
    void updateParams(const Field& field, const Team& friendly_team,
                      const Team& enemy_team, const Ball& ball);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the ball
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    /**
     * Returns true if the robot has found a shot with at least the minimum percentage of
     * the net open
     *
     * @return true if the robot has found a shot with at least the minimum percentage of
     * the net open, and false otherwise
     */
    bool hasShotAvailable() const;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    /**
     * Returns true if there is an enemy close enough to steal the ball from the shooting
     * robot
     *
     * @return true if there is an enemy close enough to steal the ball from the shooting
     * robot
     */
    bool isEnemyAboutToStealBall() const;

    /**
     * A helper function to return information about shots we may want to take. Returns an
     * optional containing a pair of the shot target to shoot at, and the fraction of the
     * net that is open to shoot at. If the optional is null / empty, there is no possible
     * shot on the enemy net.
     *
     * @return An optional pair containing the target to shoot at and the fraction of the
     * net that is open
     */
    std::optional<std::pair<Point, double>> getShotData() const;

    /**
     * A helper function that will continuously yield kick or chip intents until there is
     * no possible shot on net
     *
     * @param kick_action The kick action to run
     * @param chip_action The chip action to run
     * @param yield The coroutine to yield
     */
    void shootUntilShotBlocked(KickAction& kick_action, ChipAction& chip_action,
                               IntentCoroutine::push_type& yield) const;

    // Tactic parameters
    // The field being played on
    Field field;
    // The friendly team
    Team friendly_team;
    // The enemy team
    Team enemy_team;
    // The ball
    Ball ball;
    // A value in the range [0, 1.0] that indicates the fraction of the net that must be
    // open in order for the robot to take a shot
    double min_percent_net_open;
    // Whether or not there is currently a shot available with at least the minimum
    // percentage of the net open
    bool has_shot_available;
    // The point the robot will chip towards if it is unable to shoot and is in danger or
    // losing the ball to an enemy
    std::optional<Point> chip_target;

    // How far from the ball an enemy must be to be considered a danger that may steal the
    // ball
    const double ENEMY_DANGER_DIST = 0.5 + ROBOT_MAX_RADIUS_METERS;
    // How far we try chip when chipping over an enemy. This value is relatively small so
    // that we chip over the enemy, but don't launch the ball so far we have no chance of
    // recovering possession
    const double CHIP_DIST = 0.5;
    // The distance between the robot's kicker and the ball while tracking the ball
    // waiting for a shot
    const double TRACK_BALL_DIST = 0.05;
};
