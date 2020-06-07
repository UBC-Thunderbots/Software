#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"

class DribbleAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD = 0.02;

    /**
     * Creates a new DribbleAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     */
    explicit DribbleAction(
        double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD);

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_angle The final orientation the robot should have at
     * the destination
     * @param rpm RPM of the dribbler
     * @param small_kick_allowed If true, the robot can kick the ball forward a small
     * distance while dribbling
     */
    void updateControlParams(const Robot& robot, const Point& dest,
                             const Angle& final_angle, double rpm,
                             bool small_kick_allowed);

    void accept(MutableActionVisitor& visitor) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    Angle final_orientation;
    double dribbler_rpm;
    bool small_kick_allowed;

    double close_to_dest_threshold;
};
