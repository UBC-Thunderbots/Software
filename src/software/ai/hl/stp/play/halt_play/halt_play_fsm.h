#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/logger/logger.h"

struct HaltPlayFSM
{
    struct ControlParams
    {
    };

    class HaltState;
    class TimeoutState;

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

    /**
     *Action to make the robot make a T like formation in half time
     * @param event the HaltPlayFSM event
     */
    void updateTimeout(const Update& event);

    /**
     *  Guards on whether the FSM should enter timeout
     *
     * @param event the HaltPlayFSM event
     */
    bool shouldEnterTimeout(const Update& event);

    /**
     *  Guards on whether the FSM should enter default halt state
     *
     * @param event the HaltPlayFSM event
     */
    bool shouldEnterHalt(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(HaltState)
        DEFINE_SML_STATE(TimeoutState)

        DEFINE_SML_GUARD(shouldEnterHalt);
        DEFINE_SML_GUARD(shouldEnterTimeout);

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(updateStop)
        DEFINE_SML_ACTION(updateTimeout)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *HaltState_S + Update_E[shouldEnterHalt_G] / updateStop_A = HaltState_S,
            HaltState_S + Update_E[shouldEnterTimeout_G] / updateTimeout_A =
                TimeoutState_S,

            TimeoutState_S + Update_E[shouldEnterHalt_G] / updateStop_A = HaltState_S,
            TimeoutState_S + Update_E[shouldEnterTimeout_G] / updateTimeout_A =
                TimeoutState_S,


            // HaltState_S + Update_E / updateStop_A = HaltState_S,

            X + Update_E / updateStop_A = X);
    }

   private:
    PriorityTacticVector halt_tactics;
};
