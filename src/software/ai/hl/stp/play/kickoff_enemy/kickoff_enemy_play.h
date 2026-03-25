#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"
#include "software/ai/hl/stp/play/play_fsm.hpp"

/**
 * A play that runs when its currently the enemy kick off.
 */

class KickoffEnemyPlay : public PlayBase<KickoffEnemyPlayFSM>
{
   public:
    /**
     * Creates an enemy kickoff play
     *
     * @param ai_config_ptr the play config for this play
     */
    explicit KickoffEnemyPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
