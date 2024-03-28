#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{

public:
    EnemyFreekickPlay(TbotsProto::AiConfig config);

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

private:
    FSM<EnemyFreeKickPlayFSM> fsm;
    EnemyFreeKickPlayFSM::ControlParams control_params;
};