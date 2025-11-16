#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * Play that set up crease defenders to defend the defense area
 */
class CreaseDefensePlay : public PlayBase<CreaseDefensePlayFSM>
{
   public:
    /**
     * Constructor for CreaseDefensePlay
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    CreaseDefensePlay(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Update control params for this play
     *
     * @param enemy_threat_origin The origin of the enemy threat
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     */
    void updateControlParams(const Point &enemy_threat_origin,
                             TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
                                 TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
};
