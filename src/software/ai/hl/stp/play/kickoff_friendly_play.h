#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the friendly kick off,
 * only one robot grabs the ball and passes to another robot.
 */
class KickoffFriendlyPlay : public Play
{
   public:
    KickoffFriendlyPlay() = default;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
