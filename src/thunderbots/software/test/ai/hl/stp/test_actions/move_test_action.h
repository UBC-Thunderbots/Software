#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * A test Action used for unit tests
 *
 * Moves the given robot to the given destination
 */
class MoveTestAction : public Action
{
   public:
    /**
     * Creates a new MoveTestAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     */
    explicit MoveTestAction(double close_to_dest_threshold);

    /**
     * Returns the next Intent this MoveTestAction wants to run, given the parameters.
     * Moves the robot in a straight line to the given destination.
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     *
     * @return A unique pointer to the Intent the MoveTestAction wants to run. If the
     * MoveTestAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        Point destination);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    double close_to_dest_threshold;
};
