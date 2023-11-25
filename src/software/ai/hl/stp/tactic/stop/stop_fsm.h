#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

struct StopFSM
{
   public:
    class StopState;

    struct ControlParams
    {
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Constructor for StopFSM struct
     */
    explicit StopFSM() {}

    /**
     * Action to set the StopPrimitive
     *
     * @param event StopFSM::Update
     */
    void updateStop(const Update& event);

    /**
     * Guard if the stop is done
     *
     * @param event StopFSM::Update
     *
     * @return if the robot has stopped
     */
    bool stopDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StopState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(stopDone)
        DEFINE_SML_ACTION(updateStop)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StopState_S + Update_E[!stopDone_G] / updateStop_A = StopState_S,
            StopState_S + Update_E[stopDone_G] / updateStop_A   = X,
            X + Update_E[!stopDone_G] / updateStop_A            = StopState_S,
            X + Update_E[stopDone_G] / updateStop_A             = X);
    }
};
