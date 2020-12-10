#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

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
     * @param loop_forever restarts the action one it completes
     */
    explicit MoveTestAction(double close_to_dest_threshold, bool loop_forever);

    /**
     * Updates the params for this action that cannot be derived from the world
     */
    void updateControlParams(const Robot& robot, Point destination);

    void updateWorldParams(const World& world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    double close_to_dest_threshold;
};
