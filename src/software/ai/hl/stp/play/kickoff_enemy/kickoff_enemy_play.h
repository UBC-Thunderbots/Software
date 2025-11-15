#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A play that runs when its currently the enemy kick off.
 */

class KickoffEnemyPlay : public Play
{
public:
    /**
    * Creates an enemy kickoff play
    *
    * @param ai_config the play config for this play
    */
    KickoffEnemyPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

private:
    FSM<KickoffEnemyPlayFSM> fsm;
    KickoffEnemyPlayFSM::ControlParams control_params;
};