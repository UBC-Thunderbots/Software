#pragma once

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/tactic/tactic.h"

class MoveTactic : public Tactic
{
   public:
    explicit MoveTactic(const Robot& robot);

    /**
     * Returns the next Intent this MoveTactic wants to run, given the parameters.
     * Moves the robot in a straight line to the given destination.
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     *
     * @return A unique pointer to the Intent the MoveAction wants to run. If the
     * MoveAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        Point destination,
                                                        Angle final_orientation,
                                                        double final_speed);

    double calculateRobotCost(const Robot& robot, const Field& field) override;

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Tactic parameters
    Point destination;
    Angle final_orientation;
    double final_speed;
};
