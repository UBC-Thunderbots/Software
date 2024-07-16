#pragma once

#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"

/**
 * A play for free kicks
 */

class FreeKickPlay : public Play
{
   public:
    /**
     * Creates a free kick play
     *
     * @param strategy the Strategy shared by all of AI
     */
    FreeKickPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<FreeKickPlayFSM> fsm;
    FreeKickPlayFSM::ControlParams control_params;
};
