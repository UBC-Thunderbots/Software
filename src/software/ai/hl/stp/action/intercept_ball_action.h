#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/world/ball.h"
#include "software/world/field.h"

class InterceptBallAction : public Action
{
   public:
    /**
     * Creates a new InterceptBallAction
     *
     * @param field The field
     * @param ball The ball
     */
    explicit InterceptBallAction(const Field& field, const Ball& ball);

    InterceptBallAction() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates this action with all the parameters it needs that don't come from the world
     *
     * @param robot The robot to move
     */
    void updateControlParams(const Robot& robot);

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

    const double BALL_MOVING_SLOW_SPEED_THRESHOLD     = 0.3;
    const double BALL_CLOSE_TO_DRIBBLER_THRESHOLD     = 0.2 + BALL_MAX_RADIUS_METERS;
    const double SIMILAR_VELOCITY_MAGNITUDE_THRESHOLD = 0.05;
    const Angle SIMILAR_VELOCITY_ANGLE_THRESHOLD      = Angle::fromDegrees(10);
    const double ROBOT_STOPPED_SPEED_M_PER_S          = 0.03;
    const double INTERCEPT_POSITION_SEARCH_INTERVAL   = 0.1;

    // Action parameters
    Field field;
    Ball ball;
};
