#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

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
    bool done() const override;
    void updateTactics(const PlayUpdate &play_update) override;

   private:
    FSM<ShootOrPassPlayFSM> fsm;
    ShootOrPassPlayFSM::ControlParams control_params;
};
