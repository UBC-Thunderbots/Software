#pragma once

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
 * The ShadowFreekicker tactic uses two robots to block the majority of an enemy robots
 *
 */
class ShadowFreekickerTactic : public Tactic
{
   public:
    enum FreekickShadower
    {
        First  = 0,
        Second = 1,
    };

    /**
     * Creates a new ShadowFreekicker tactic
     * @param free_kick_shadower : Enum selection of the left/right Freekicker shadower
     * @param enemy_team : The enemy team of robots
     * @param ball : Ball object
     * @param field : Field the robots are playing on
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ShadowFreekickerTactic(FreekickShadower free_kick_shadower, Team enemy_team,
                                    Ball ball, Field field, bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this ShadowFreekicker tactic
     *
     * @param enemy_team : The enemy team of robots.
     * @param ball  : The Ball being played with
     */
    void updateParams(Team enemy_team, Ball ball);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Used for defining whether this robot is a left/right Freekick Shadower
    FreekickShadower free_kick_shadower;
    Ball ball;
    Field field;
    Team enemy_team;

    const double FREE_KICK_MAX_PROXIMITY =
        0.50 * 1.05;  // Robots cannot be closer than 50cm from the ball during a freekick
                      // (with a buffer factor)
};
