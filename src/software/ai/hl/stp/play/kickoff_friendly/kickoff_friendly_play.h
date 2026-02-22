#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/play/play_base.hpp"
#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the friendly kick off,
 * only one robot grabs the ball and passes to another robot.
 */

class KickoffFriendlyPlay : public PlayBase<KickoffFriendlyPlayFSM>
{
   public:
    /**
     * Creates a friendly kickoff play
     *
     * @param ai_config_ptr the play config for this play
     */
    explicit KickoffFriendlyPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

};
