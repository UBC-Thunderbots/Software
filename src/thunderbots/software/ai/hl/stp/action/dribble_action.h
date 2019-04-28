#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

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
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit DribbleAction(double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD,
                        bool loop_forever              = false);

    /**
     * Returns the next Intent this DribbleAction wants to run, given the parameters.
     * Moves the robot in a straight line to the given destination.
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param enable_autokick This will enable the "break-beam" on the robot, that will
     *                        trigger the kicker to fire as soon as the ball is in front
     *                        of it
     *
     * @return A unique pointer to the Intent the DribbleAction wants to run. If the
     * DribbleAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(
        const Robot& robot, const Point &dest,
                             const Angle &final_angle, double rpm,
                             bool small_kick_allowed);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    Angle final_orientation;
    double dribbler_rpm;
    bool small_kick_allowed;

    double close_to_dest_threshold;
    bool loop_forever;
};
