#pragma once

#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/intent/autokick_move_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

/**
 * The AutokickMoveAction makes the robot move to the specified location with autokick
 * armed
 *
 * Done condition: The robot is close to the specified location
 */
class AutokickMoveAction : public MoveAction
{
   public:
    /**
     * Creates a new AutokickMoveAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit AutokickMoveAction(
        bool loop_forever, double close_to_dest_threshold = ROBOT_CLOSE_TO_DEST_THRESHOLD,
        Angle close_to_orientation_threshold = ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);

    AutokickMoveAction() = delete;

    /**
     * Gets kick speed
     *
     * @return the kick speed in meters per seconds
     */
    double getKickSpeed();

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param dribbler_mode The dribbler mode
     * @param kick_speed_meters_per_second The kick speed in meters per second
     * @param ball_collision_type how to navigate around the ball
     */
    void updateControlParams(const Robot& robot, Point destination,
                             Angle final_orientation, double final_speed,
                             DribblerMode dribbler_mode,
                             double kick_speed_meters_per_second,
                             BallCollisionType ball_collision_type);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    double kick_speed_meters_per_second_;
};
