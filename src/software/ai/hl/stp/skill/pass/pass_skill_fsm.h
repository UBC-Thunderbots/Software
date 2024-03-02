#pragma once

#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PassSkillFSM
{
    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    void takePass(const Update& event, boost::sml::back::process<PivotKickSkillFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PivotKickSkillFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(takePass, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PivotKickSkillFSM_S + Update_E / takePass_A, PivotKickSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
