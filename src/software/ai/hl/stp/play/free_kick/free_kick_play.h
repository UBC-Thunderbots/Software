#pragma once

#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"

/**
 * This free kick play is used for free kicks, which are used to restart the game after a
 * foul has occurred. Additionally, goal kicks and corner kicks are mapped to free kicks.
 * The logic of this play is:
 * - One robot (the kicker) attempts to shoot first. If there is a good shot, then it
 * will shoot the ball.
 * - If there is no good shot, the kicker will attempt to pass. Two robots try to get in
 * good positions near the enemy net to receive a pass
 * - If we cannot find a pass in time, the kicker will chip the ball towards the enemy
 * goal
 * - Two robots crease defend
 * - One robot is goalie
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
