#pragma once

#include "ai/hl/stp/action/dribble_action.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
 * The DribbleTactic will move the assigned robot to the given destination and arrive
 * with the specified final orientation and speed
 */
class DribbleTactic : public Tactic
{
   public:
    /**
     * Creates a new DribbleTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit DribbleTactic(bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this DribbleTactic.
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     */
    void updateParams(const Point &dest,
                             const Angle &final_angle, double rpm,
                             bool small_kick_allowed);

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
    // The orientation the robot should have when it arrives at its destination
    Angle final_orientation;
    double dribbler_rpm;
    bool small_kick_allowed;
};
