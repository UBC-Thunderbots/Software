#pragma once

#include "ai/hl/stp/tactic/tactic.h"

/**
 * A test Tactic used for unit tests
 *
 * Stops the robot from moving
 */
class StopTestTactic : public Tactic
{
   public:
    /**
     * Creates a new StopTestTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit StopTestTactic(bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this StopTestTactic.
     */
    void updateParams();

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally with a cost of 0.5
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;
};
