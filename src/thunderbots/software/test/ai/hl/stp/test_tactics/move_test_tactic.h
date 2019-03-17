#pragma once

#include "ai/hl/stp/tactic/tactic.h"

/**
 * A test Tactic used for unit tests
 *
 * Moves the assigned robot to the given destination
 */
class MoveTestTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveTestTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit MoveTestTactic(bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this MoveTestTactic.
     *
     * @param destination The destination to move to (in global coordinates)
     */
    void updateParams(Point destination);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
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

    // Tactic parameters
    // The point the robot is trying to move to
    Point destination;
};
