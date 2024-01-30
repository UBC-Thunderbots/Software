#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending penalty kicks
 */
class PenaltyKickEnemyPlay : public Play
{
   public:
    PenaltyKickEnemyPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<PenaltyKickEnemyPlayFSM> fsm;
    PenaltyKickEnemyPlayFSM::ControlParams control_params;
};
