#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"

struct StopSkillFSM
{
   public:
    class StopState;

    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action to set the StopPrimitive
     *
     * @param event StopSkillFSM::Update
     */
    void updateStop(const Update& event);

    /**
     * Guard to check if the stop is done
     *
     * @param event StopSkillFSM::Update
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
