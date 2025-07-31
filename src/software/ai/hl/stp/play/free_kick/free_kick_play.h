#pragma once

#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play_base.h"

/**
 * A play for free kicks
 */

class FreeKickPlay : public PlayBase<FreeKickPlayFSM>
{
   public:
    /**
     * Creates a free kick play
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    FreeKickPlay(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
