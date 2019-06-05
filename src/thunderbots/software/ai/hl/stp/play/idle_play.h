#pragma once

#include "ai/hl/stp/play/play.h"

/**
 */

class IdlePlay : public Play
{
   public:
    static const std::string name;

    IdlePlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;
};
