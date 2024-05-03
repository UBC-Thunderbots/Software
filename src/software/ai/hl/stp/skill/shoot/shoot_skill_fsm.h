#pragma once

#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct ShootSkillFSM
{
    struct GetBallControlFSM
    {
        struct ControlParams
        {
        };

        DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

        /**
         * Action that updates the DribbleSkillFSM to get control of the ball
         * and steady it
         *
         * @param event the Update event
         * @param processEvent processes the DribbleSkillFSM::Update event
         */
        void getBallControl(
            const Update& event,
            boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

        auto operator()()
        {
            using namespace boost::sml;

            DEFINE_SML_STATE(DribbleSkillFSM)
            DEFINE_SML_EVENT(Update)
            DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControl, DribbleSkillFSM)

            return make_transition_table(
                // src_state + event [guard] / action = dest_state
                *DribbleSkillFSM_S + Update_E / getBallControl_A, DribbleSkillFSM_S = X,
                X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
        }
    };

    struct ControlParams
    {
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the GetBallControlFSM to get control of the ball
     * and steady it
     *
     * @param event the Update event
     * @param processEvent processes the GetBallControlFSM::Update event
     */
    void getBallControl(
        const Update& event,
        boost::sml::back::process<GetBallControlFSM::Update> processEvent);

    /**
     * Action that updates the DribbleSkillFSM to dribble the ball to the
     * kick origin
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void dribbleBallToKickOrigin(
        const Update& event,
        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

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

        DEFINE_SML_STATE(GetBallControlFSM)
        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(PivotKickSkillFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControl, GetBallControlFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(dribbleBallToKickOrigin, DribbleSkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivotKick, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBallControlFSM_S + Update_E / getBallControl_A,
            GetBallControlFSM_S = DribbleSkillFSM_S,

            DribbleSkillFSM_S + Update_E / dribbleBallToKickOrigin_A,
            DribbleSkillFSM_S = PivotKickSkillFSM_S,

            PivotKickSkillFSM_S + Update_E / pivotKick_A, PivotKickSkillFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<Shot> best_shot_ = std::nullopt;
};
