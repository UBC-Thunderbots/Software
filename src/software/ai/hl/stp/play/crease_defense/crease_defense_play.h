#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that set up crease defenders to defend the defense area
 */
class CreaseDefensePlay : public Play
{
   public:
    CreaseDefensePlay(TbotsProto::AiConfig config, std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void reset(const TbotsProto::AiConfig& ai_config) override;
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

   private:
    std::unique_ptr<FSM<CreaseDefensePlayFSM>> fsm;
    CreaseDefensePlayFSM::ControlParams control_params;
};
