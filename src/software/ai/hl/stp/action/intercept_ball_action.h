#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
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
     * Moves the robot to intercept a slow moving ball by running into
     * it with the dribbler and bringing it to a stop.
     *
     * @param yield The coroutine for the action
     */
    void interceptSlowBall(IntentCoroutine::push_type& yield);

    /**
     * Moves the robot to intercept a fast moving ball by predicting its future
     * location and moving there to catch it.
     *
     * @param yield The coroutine for the action
     */
    void interceptFastBall(IntentCoroutine::push_type& yield);

    const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;

    // Action parameters
    Field field;
    Ball ball;
};
