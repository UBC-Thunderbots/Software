#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't, while keeping some robots
 * to protect the defense area
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<Robot> getInjuredRobots(const World &world) override;

   private:
    FSM<OffensePlayFSM> fsm;
    OffensePlayFSM::ControlParams control_params;
};
