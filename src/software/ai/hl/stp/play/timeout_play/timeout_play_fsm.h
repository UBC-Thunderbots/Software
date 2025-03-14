#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/logger/logger.h"

struct TimeoutPlayFSM
{
    struct ControlParams
    {
    };

    class TimeoutState;

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a Halt Play FSM
     *
     * @param ai_config the play config for this FSM
     */
    explicit TimeoutPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     *Action to make the robot make a T like formation in half time
     * @param event the TimeoutPlayFSM event
     */
    void updateTimeout(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(TimeoutState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateTimeout)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *TimeoutState_S + Update_E / updateTimeout_A = TimeoutState_S,
            X + Update_E / updateTimeout_A               = X
            );
    }

   private:
};
