#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that sets up defenders to block shots on net and intercept
 * passes between enemy robots
 */
class DefensePlay : public Play
{
   public:
    DefensePlay(const TbotsProto::AiConfig &config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Update control params for this play
     *
     * @param max_allowed_speed_mode the mode of maximum speed allowed
     */
    void updateControlParams(std::queue<DefenderAssignment> &defender_assignments, TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode=TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

   private:
    FSM<DefensePlayFSM> fsm;
    DefensePlayFSM::ControlParams control_params;
};
