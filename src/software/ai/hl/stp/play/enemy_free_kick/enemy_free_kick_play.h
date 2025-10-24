#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * Play for defending against enemy free kicks, attempting to block the kicker, while also
 * protected potential passes
 */
class EnemyFreeKickPlay : public PlayBase<EnemyFreeKickPlayFSM>
{
   public:
    /**
     * Constructor for EnemyFreeKickPlay to defend against enemy free kicks, attempting to
     * block the kicker
     * @param ai_config_ptr shared pointer to ai_config
     */
    EnemyFreeKickPlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Update control params for this play
     *
     * @param max_allowed_speed_mode the mode of maximum speed allowed
     */
    void updateControlParams(TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
                                 TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
};
