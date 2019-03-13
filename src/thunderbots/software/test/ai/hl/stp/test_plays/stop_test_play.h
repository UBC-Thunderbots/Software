#pragma once

#include "ai/hl/stp/play/play.h"

/**
 * A test Play that stops 3 robots.
 *
 * The Play is never done
 *
 * This play is applicable when the ball's y coordinate is >= 0
 * This play's invariant holds while the ball is within the field
 *
 * We use the Ball's position to control the 'isApplicable' and 'invariantHolds' values
 * because it is easy to change during tests
 */
class StopTestPlay : public Play
{
   public:
    static const std::string name;

    StopTestPlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    std::vector<std::shared_ptr<Tactic>> getNextTactics(TacticCoroutine::push_type &yield,
                                                        const World &world) override;
};
