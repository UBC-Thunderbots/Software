#pragma once

#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct ShootSkillFSM
{
    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the PivotKickSkillFSM to shoot
     *
     * @param event the Update event
     * @param processEvent processes the PivotKickSkillFSM::Update event
     */
    void pivotKick(const Update& event,
                   boost::sml::back::process<PivotKickSkillFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PivotKickSkillFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivotKick, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PivotKickSkillFSM_S + Update_E / pivotKick_A, PivotKickSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
