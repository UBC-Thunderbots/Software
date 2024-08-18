#pragma once

#include <memory>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/strategy.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    /**
     * Creates a PenaltyKickPlay
     *
     * @param strategy the Strategy shared by all of AI
     */
    PenaltyKickPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    std::unique_ptr<FSM<PenaltyKickPlayFSM>> fsm;
    PenaltyKickPlayFSM::ControlParams control_params;
};
