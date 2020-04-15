#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/ai/hl/stp/action/mutable_action_visitor.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/move_primitive.h"
#include "software/world/ball.h"
#include "software/world/field.h"

class InterceptBallAction : public Action
{
   public:
    /**
     * Creates a new InterceptBallAction
     *
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit InterceptBallAction(const Field& field, const Ball& ball, bool loop_forever);

    /**
     * Updates this action with all the parameters it needs from the world
     *
     * @param field The field
     * @param ball The ball
     */
    void updateWorldParams(const Field& field, const Ball& ball);

    /**
     * Updates this action with all the parameters it needs that don't come from the world
     *
     * @param robot The robot to move
     */
    void updateControlParams(const Robot& robot);

    void accept(MutableActionVisitor& visitor) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    /**
     * Determines the best place to intercept the ball and move the robot to that
     * position.
     *
     * @param yield The coroutine to yield to
     * @param closest_point_on_ball_trajectory The closest point on the ball's trajectory
     * to the robot's current position
     */
    void moveToInterceptPosition(IntentCoroutine::push_type& yield,
                                 Point closest_point_on_ball_trajectory);

    /**
     * Returns the point at which the ball would leave the field given its current
     * trajectory. This function assumes the ball's current position is within the field.
     * If it is not, an std::nullopt is returned.
     *
     * @param field The field
     * @param ball The ball
     * @return An optional containing the point at which the ball would leave the field.
     * If the ball's position is not within the field, std::nullopt is returned
     */
    std::optional<Point> getPointBallLeavesField(const Field& field, const Ball& ball);

    const double BALL_MOVING_SLOW_SPEED_THRESHOLD              = 0.3;
    const double ROBOT_CLOSE_TO_BALL_TRAJECTORY_LINE_THRESHOLD = 0.02;
    const double FINAL_SPEED_AT_SLOW_BALL                      = 0.3;

    // Action parameters
    Field field;
    Ball ball;
};
