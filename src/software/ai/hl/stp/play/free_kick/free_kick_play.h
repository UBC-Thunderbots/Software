#pragma once

#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"

/**
 * A Play for Direct Free kicks
 */
class FreeKickPlay : public Play
{
   public:
    FreeKickPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<FreeKickPlayFSM> fsm;
    FreeKickPlayFSM::ControlParams control_params;
};
