#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

class MoveAction : public Action
{
   public:
    /**
     * Creates a new MoveAction
     */
    explicit MoveAction(const Robot& robot);

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
    std::unique_ptr<Intent> updateStateAndGetNextIntent(Robot robot, Point destination,
                                                        Angle final_orientation,
                                                        double final_speed);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Robot robot;
    Point destination;
    Angle final_orientation;
    double final_speed;

    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    const double ROBOT_CLOSE_TO_DEST = 0.02;

    // Variables to sore the Intent the Action wants to run, and the coroutine pull_type
    std::unique_ptr<Intent> curr_intent;
    intent_coroutine::pull_type intent_sequence;
};