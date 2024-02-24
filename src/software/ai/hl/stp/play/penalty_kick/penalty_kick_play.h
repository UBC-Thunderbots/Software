#pragma once

#include <memory>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    PenaltyKickPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    std::unique_ptr<FSM<PenaltyKickPlayFSM>> fsm;
    PenaltyKickPlayFSM::ControlParams control_params;
};
