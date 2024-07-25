#pragma once

#include "software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct ShootSkillFSM
{
    struct ControlParams
    {
        // Sample multiple potential shot origin points when finding
        // the best shot to take (true), or do not sample (false)
        bool sample_for_best_shot;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Guard that updates the current best shot on goal and checks whether we 
     * should abort the shot because it is unlikely to succeed.
     *
     * @param event the Update event
     *
     * @return true if we should abort the shot, false otherwise
     */
    bool shouldAbortShot(const Update& event);

    /**
     * Action that updates the DribbleSkillFSM to get control of the ball
     * and steady it
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void getBallControl(const Update& event,
                        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Action that updates the PivotKickSkillFSM to shoot
     *
     * @param event the Update event
     * @param processEvent processes the PivotKickSkillFSM::Update event
     */
    void pivotKick(const Update& event,
                   boost::sml::back::process<PivotKickSkillFSM::Update> processEvent);

    /**
     * Action that aborts the current shot, stopping the robot and
     * resetting the SkillState
     *
     * @param event the Update event
     */
    void abortShot(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(PivotKickSkillFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldAbortShot)

        DEFINE_SML_ACTION(abortShot)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControl, DribbleSkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivotKick, PivotKickSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleSkillFSM_S + Update_E / getBallControl_A,
            DribbleSkillFSM_S = PivotKickSkillFSM_S,

            PivotKickSkillFSM_S + Update_E[shouldAbortShot_G] / abortShot_A = X,
            PivotKickSkillFSM_S + Update_E / pivotKick_A, PivotKickSkillFSM_S = X,

            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<Shot> best_shot_ = std::nullopt;
};
