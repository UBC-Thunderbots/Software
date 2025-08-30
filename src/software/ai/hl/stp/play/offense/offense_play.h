#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public PlayBase<OffensePlayFSM>
{
   public:
    /**
     * Constructor for OffensePlay
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    OffensePlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
};
