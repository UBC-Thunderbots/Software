#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

struct HaltFSM
{
   public:
    class StopState;

    struct ControlParams
    {
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Constructor for HaltFSM struct
     */
    explicit HaltFSM() {}

    /**
     * Action to set the StopPrimitive
     *
     * @param event HaltFSM::Update
     */
    void updateHalt(const Update& event);

    /**
     * Guard if the stop is done
     *
     * @param event HaltFSM::Update
     *
     * @return if the robot has stopped
     */
    bool haltDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StopState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(haltDone)
        DEFINE_SML_ACTION(updateHalt)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StopState_S + Update_E[!haltDone_G] / updateHalt_A = StopState_S,
            StopState_S + Update_E[haltDone_G] / updateHalt_A   = X,
            X + Update_E[!haltDone_G] / updateHalt_A            = StopState_S,
            X + Update_E[haltDone_G] / updateHalt_A             = X);
    }
};
