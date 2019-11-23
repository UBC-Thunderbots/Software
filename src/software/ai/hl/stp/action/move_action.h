#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/hl/stp/action/action_visitor.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/primitive/move_primitive.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"

class MoveAction : public Action
{
   public:
    // We consider the robot close to a destination when it is within 2 cm.
    // The robot should be able to reach within 1cm of the destination even with
    // camera and positioning noise
    static constexpr double ROBOT_CLOSE_TO_DEST_THRESHOLD       = 0.02;
    static constexpr Angle ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD = Angle::fromDegrees(2);
    /**
     * Creates a new MoveAction
     *
     * @param close_to_dest_threshold How far from the destination the robot must be
     * before the action is considered done
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit MoveAction(
        double close_to_dest_threshold       = ROBOT_CLOSE_TO_DEST_THRESHOLD,
        Angle close_to_orientation_threshold = ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD,
        bool loop_forever                    = false);

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot to move
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param slow Whether or not to move slow
     * @param autokick This will enable the "break-beam" on the robot, that will
     * trigger the kicker or chippper to fire as soon as the ball is in front of it
     * @param ball_collision_type how to navigate around the ball
     */
    void updateControlParams(const Robot& robot, Point destination,
                             Angle final_orientation, double final_speed,
                             DribblerEnable enable_dribbler, MoveType move_type,
                             AutokickType autokick,
                             BallCollisionType ball_collision_type);

    void accept(ActionVisitor& visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Point destination;
    Angle final_orientation;
    double final_speed;
    DribblerEnable enable_dribbler;
    MoveType move_type;
    AutokickType autokick;
    BallCollisionType ball_collision_type;

    double close_to_dest_threshold;
    Angle close_to_orientation_threshold;
    bool loop_forever;
};
