#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/offensive_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't.
 */
class ShootOrPassPlay : public Play
{
   public:
    ShootOrPassPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    void updateTactics(const PlayUpdate &play_update) override;

    FSM<OffensivePlayFSM> offensive_fsm;
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics;
};
