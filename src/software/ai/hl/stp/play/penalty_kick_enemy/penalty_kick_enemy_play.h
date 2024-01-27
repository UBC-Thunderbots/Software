#pragma once

#include <memory>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"

/**
 * Play for defending penalty kicks
 */
class PenaltyKickEnemyPlay : public Play
{
   public:
    PenaltyKickEnemyPlay(const TbotsProto::AiConfig &config,
                         std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void reset() override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    std::unique_ptr<FSM<PenaltyKickEnemyPlayFSM>> fsm;
    PenaltyKickEnemyPlayFSM::ControlParams control_params;
};
