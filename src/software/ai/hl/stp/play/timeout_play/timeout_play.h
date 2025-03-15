#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/timeout_play/timeout_play_fsm.h"

/**
 *  A play that represents timeout formation. This would make the format
 *  make a t like formation
 */
class TimeoutPlay : public Play
{
   public:
    TimeoutPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;

   private:
    FSM<TimeoutPlayFSM> fsm;
    TimeoutPlayFSM::ControlParams control_params;
};
