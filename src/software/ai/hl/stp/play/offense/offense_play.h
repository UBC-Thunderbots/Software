#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    bool done() const override;

   private:
    void updateTactics(const PlayUpdate &play_update) override;

    FSM<OffensePlayFSM> fsm;
};
