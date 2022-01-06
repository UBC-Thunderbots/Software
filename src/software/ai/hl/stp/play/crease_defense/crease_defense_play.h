#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that set up crease defenders to defend the defense area
 */
class CreaseDefensePlay : public Play
{
   public:
    CreaseDefensePlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;
    bool invariantHolds(const World &world) const override;
    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    bool done() const override;

    /**
     * Update control params for this play
     *
     * @param enemy_threat_origin The origin of the enemy threat
     * @param num_additional_crease_defenders_tactics The number of additional crease
     * defenders
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     */
    void updateControlParams(
        const Point &enemy_threat_origin,
        unsigned int num_additional_crease_defenders_tactics,
        MaxAllowedSpeedMode max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT);

   private:
    void updateTactics(const PlayUpdate &play_update) override;

    FSM<CreaseDefensePlayFSM> fsm;
    CreaseDefensePlayFSM::ControlParams control_params;
};
