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
    /*
     * Create timeout play
     *
     * @param config the tbots ai config
     */
    TimeoutPlay(TbotsProto::AiConfig config);

    /*
     * This function doesn't get called and will be removed when coroutines are phased
     * out.
     */
    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    /*
     * Updating tactics
     *
     * @param play_update the play update
     */
    void updateTactics(const PlayUpdate &play_update) override;

   private:
    /*
     * Making timeout formation
     *
     * @param word the world where the field is in
     * @returns   a tactic vector representing the timeout formation
     */
    PriorityTacticVector makeTimeoutFormation(WorldPtr world);

    FSM<TimeoutPlayFSM> fsm;
    TimeoutPlayFSM::ControlParams control_params;
};
