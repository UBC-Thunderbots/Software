#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A test Play that moves a robot to the friendly goal, a robot to the enemy goal, and
 * a robot to the center of the field.
 *
 * The Play is done when the robot reaches the center of the field (within 5cm)
 *
 * This play is applicable when the ball's x and y coordinates are >= 0
 * This play's invariant holds while the ball's x coordinate is >= 0
 */
class MoveTestPlay : public Play
{
   public:
    MoveTestPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    TbotsProto::AiConfig ai_config;
};
