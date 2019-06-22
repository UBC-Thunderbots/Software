#pragma once

#include "ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the friednly kick off,
 * only one robot grabs the ball and passes to another robot.
 */
class KickoffFriendlyPlay : public Play
{
   public:
    static const std::string name;

    KickoffFriendlyPlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;
};
