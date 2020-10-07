#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * A test Play that halts 3 robots.
 *
 * The Play is never done
 *
 * This play is applicable when the ball's y coordinate is >= 0
 * This play's invariant holds while the ball is within the field
 *
 * We use the Ball's position to control the 'isApplicable' and 'invariantHolds' values
 * because it is easy to change during tests
 */
class HaltTestPlay : public Play
{
   public:
    static const std::string name;

    HaltTestPlay() = default;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
