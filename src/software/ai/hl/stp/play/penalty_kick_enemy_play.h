#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending penalty kicks
 */
class PenaltyKickEnemyPlay : public Play
{
   public:
    static const std::string name;

    PenaltyKickEnemyPlay() = default;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
