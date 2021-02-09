#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * A test Play that moves a robot to the friendly goal, a robot to the enemy goal, and
 * a robot to the center of the field.
 *
 * The Play is done when the robot reaches the center of the field (within 5cm)
 *
 * This play is applicable when the ball's x and y coordinates are >= 0
 * This play's invariant holds while the ball's x coordinate is >= 0
 *
 * We use the Ball's position to control the 'isApplicable' and 'invariantHolds' values
 * because it is easy to change during tests
 */
class MoveTestPlay : public Play
{
   public:
    MoveTestPlay() ;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
