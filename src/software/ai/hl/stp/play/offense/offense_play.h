#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * OffensePlay is a Play that is run when we have possession of the ball.
 * It is a Play that sets up tactics to make passes and score goals.
 *
 * The main ball handler in OffensePlay is AttackerTactic. OffensePlay enables
 * coordination between the AttackerTactic and ReceiverTactic by relaying
 * information between them.
 */
class OffensePlay : public Play
{
   public:
    OffensePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;

    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Terminates the play, ending the current episode.
     *
     * @param world_ptr the final World of the current episode
     */
    void terminate(const WorldPtr &world_ptr);

   private:
    FSM<OffensePlayFSM> fsm;
    OffensePlayFSM::ControlParams control_params;
};
