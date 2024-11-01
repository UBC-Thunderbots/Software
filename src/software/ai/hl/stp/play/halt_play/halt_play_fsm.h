#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/logger/logger.h"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/ai/hl/stp/play/play_fsm.h"

struct HaltPlayFSM{
    struct ControlParams
    {
    };
    class HaltState;

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a Halt Play FSM
     *
     * @param ai_config the play config for this FSM
     */
    explicit HaltPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Action to make each robot stop
     *
     * @param event the HaltPlayFSM Event
     */
    void updateStop(const Update& event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(HaltState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateStop)

        return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *HaltState_S + Update_E / updateStop_A = HaltState_S,

                X + Update_E /updateStop_A             = X);
    }

    private:
        PriorityTacticVector halt_tactics;

};