#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't.
 */
class ShootOrPassPlay : public Play
{
   public:
    ShootOrPassPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<ShootOrPassPlayFSM> fsm;
    ShootOrPassPlayFSM::ControlParams control_params;
};
