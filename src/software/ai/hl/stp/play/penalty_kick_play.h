#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    PenaltyKickPlay() = default;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
