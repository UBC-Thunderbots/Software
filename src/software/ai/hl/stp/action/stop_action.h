#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/hl/stp/action/mutable_action_visitor.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"

/**
 * The StopAction makes the robot stop with the option to coast
 */
class StopAction : public Action
{
   public:
    // We consider the robot to be stopped when the magnitude of its velocity is less than
    // 5cm / s When robots are stationary, noise in the camera data can cause its velocity
    // to be non-zero, which is why we use a non-zero value here.
    static constexpr double ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT = 0.05;

    /**
     * Creates a new StopAction
     *
     * @param stopped_speed_threshold How slow the robot must be moving before the action
     * is considered done
     */
    explicit StopAction(bool loop_forever, double stopped_speed_threshold =
                                               ROBOT_STOPPED_SPEED_THRESHOLD_DEFAULT);

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot the robot that should stop
     * @param coast Whether or not to coast to a stop. If this is false, the robot will
     * try actively brake to come to a stop
     */
    void updateControlParams(const Robot& robot, bool coast);

    void accept(MutableActionVisitor& visitor) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    // Whether or not the robot should coast to a stop
    bool coast;
    // The maximum speed the robot may be moving at to be considered stopped
    double stopped_speed_threshold;
};
