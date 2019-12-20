#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"

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
     * Updates the params for this action that cannot be derived from the world
     */
    void updateControlParams(const Robot& robot, Point destination);

    void accept(ActionVisitor& visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    double close_to_dest_threshold;
};
