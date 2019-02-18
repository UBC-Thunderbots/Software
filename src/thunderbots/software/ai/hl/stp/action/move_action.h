#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

class MoveAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD = 0.02;

    /**
     * Creates a new MoveAction
     *
     * @param robot The robot that will be performing the action
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     */
    explicit MoveAction(const Robot& robot,
                        double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD);

    /**
     * Returns the next Intent this MoveAction wants to run, given the parameters.
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

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    Angle final_orientation;
    double final_speed;
    double close_to_dest_threshold;
};
