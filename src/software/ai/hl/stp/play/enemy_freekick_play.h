#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{
   public:
    static const std::string name;

    EnemyFreekickPlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
