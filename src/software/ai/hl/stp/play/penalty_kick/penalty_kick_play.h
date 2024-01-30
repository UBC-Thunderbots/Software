#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public Play
{
   public:
    PenaltyKickPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<PenaltyKickPlayFSM> fsm;
    PenaltyKickPlayFSM::ControlParams control_params;
};
