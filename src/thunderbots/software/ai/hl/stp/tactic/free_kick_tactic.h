#pragma once

#include "ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is used when performing a free kick, and no pass is available.
 * It is also assumed that the ball is stationary, in preparation for a set piece.
 *
 * For now, it attempts to find a shot towards the enemy goal;
 * if none are found, it will chip to an open area.
 * If that isn't possible, we shoot at an enemy and hope for a good deflection.
 */

class FreeKickTactic : public Tactic
{
   public:
    /**
     * Constructs new FreeKickTactic from World
     */
    explicit FreeKickTactic(const World& world, bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this tactic
     *
     * @param world The current state of the world
     */
    void updateParams(const World& updated_world);


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

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic params
    World world;
};
