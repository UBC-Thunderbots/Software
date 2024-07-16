#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct KeepAwaySkillFSM
{
    struct ControlParams
    {
        // Whether to terminate the FSM once possession is no longer
        // threatened by the enemy team (true), or to never terminate (false)
        bool terminate_if_unthreatened = true;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Guard that checks if we should keep the ball away
     *
     * @param event the Update event
     *
     * @return if we should keep the ball away
     */
    bool shouldKeepAway(const Update& event);

    /**
     * Action that updates the DribbleSkillFSM to keep the ball away
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void keepAway(const Update& event,
                  boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(shouldKeepAway)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(keepAway, DribbleSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleSkillFSM_S + Update_E[!shouldKeepAway_G] / SET_STOP_PRIMITIVE_ACTION =
                X,
            DribbleSkillFSM_S + Update_E / keepAway_A, DribbleSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
