#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * Play for defending penalty kicks
 */
class PenaltyKickEnemyPlay : public PlayBase<PenaltyKickEnemyPlayFSM>
{
   public:
    /**
     * Constructor for Penalty Kick Enemy Play
     * @param ai_config_ptr shared pointer to ai_config
     */
    PenaltyKickEnemyPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
