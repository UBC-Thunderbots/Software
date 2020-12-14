#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

/**
 * The SpinningMoveAction will move the given Robot to the specified destination while
 * spinning with the desired angular velocity
 */
class SpinningMoveAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD = 0.02;

    /**
     * Creates a new SpinningMoveAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     * @param loop_forever restart the action after completion
     */
    explicit SpinningMoveAction(bool loop_forever, double close_to_dest_threshold =
                                                       ROBOT_CLOSE_TO_DEST_THRESHOLD);

    SpinningMoveAction() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param angular_velocity The how fast the robot should spin, in radians per second.
     * A positive value spins counterclockwise, and a negative value spins clockwise
     * @param final_linear_speed The final speed that robot should have at the final
     * destination
     */
    void updateControlParams(const Robot& robot, Point destination,
                             AngularVelocity angular_velocity, double final_linear_speed);

    /**
     * Gets the destination this SpinningMoveAction is trying to move to
     *
     * @return The destination this SpinningMoveAction is trying to move to
     */
    Point getDestination();

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    AngularVelocity angular_velocity;
    double final_linear_speed;
    double close_to_dest_threshold;
};
