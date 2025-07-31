#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.h"

/**
 * Play for shooting penalty kicks
 */
class PenaltyKickPlay : public PlayBase<PenaltyKickPlayFSM>
{
   public:
    /**
     * Constructor for PenaltyKickPlay
     * @param  ai_config_ptr shared pointer to ai_config
     */
    PenaltyKickPlay(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
