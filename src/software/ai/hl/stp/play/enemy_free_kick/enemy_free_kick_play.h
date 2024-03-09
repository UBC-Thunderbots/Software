#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h"

/**
 * Play for defending against enemy free kicks
 */
class EnemyFreekickPlay : public Play
{
public:
    EnemyFreekickPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;

public:
    OffensePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;

private:
    FSM<EnemyFreeKickPlayFSM> fsm;
    EnemyFreeKickPlayFSM::ControlParams control_params;
};