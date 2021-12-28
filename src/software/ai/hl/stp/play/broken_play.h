#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

/**
 * The Defense Play tries to grab the ball from the enemy that has it, and all other
 * robots shadow the enemy robots in order of how threatening they are.
 */
class BrokenPlay : public Play
{
   public:
    BrokenPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
