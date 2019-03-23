#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * The MoveSpinAction will move the given Robot to the specified destination while
 * spinning with the desired angular velocity
 */
class MoveSpinAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD = 0.02;

    /**
     * Creates a new MoveSpinAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     */
    explicit MoveSpinAction(
        double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD);

    /**
     * Returns the next Intent this MoveSpinAction wants to run, given the parameters.
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param angular_velocity The how fast the robot should spin, in radians per second.
     * A positive value spins counterclockwise, and a negative value spins clockwise
     *
     * @return A unique pointer to the Intent the MoveSpinAction wants to run. If the
     * MoveSpinAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        Point destination,
                                                        AngularVelocity angular_velocity);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    AngularVelocity angular_velocity;
    double close_to_dest_threshold;
};
