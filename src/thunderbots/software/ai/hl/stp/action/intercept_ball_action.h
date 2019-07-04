#pragma once

#include "ai/hl/stp/action/action.h"
#include "ai/primitive/move_primitive.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "geom/angle.h"
#include "geom/point.h"

class InterceptBallAction : public Action
{
   public:
    /**
     * Creates a new InterceptBallAction
     *
     * @param loop_forever Continue yielding new Move Intents, even after we have reached
     *                     our goal
     */
    explicit InterceptBallAction(const Field& field, const Ball& ball,
                                 bool loop_forever = false);

    /**
     * Returns the next Intent this InterceptBallAction wants to run, given the
     * parameters. Moves the robot to intercept and gain control of the ball
     *
     * @param robot The robot to move
     * @param field The field
     * @param ball The ball
     *
     * @return A unique pointer to the Intent the InterceptBallAction wants to run. If the
     * InterceptBallAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        const Field& field,
                                                        const Ball& ball);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    std::optional<Point> getPointBallLeavesField(const Field& field, const Ball& ball);

    // Action parameters
    Field field;
    Ball ball;
    bool loop_forever;
};
