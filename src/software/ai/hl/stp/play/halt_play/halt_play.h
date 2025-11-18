#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * A Play that stops all the robots on the field. This is primarily used to obey the
 * referee "Halt" command, as well as a fallback for when we don't have a play assigned.
 */
class HaltPlay : public PlayBase<HaltPlayFSM>
{
   public:
    /**
     * Constructor for Halt Play
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    HaltPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
};
