#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PivotKickSkillFSM
{
    class KickState;
    class StartState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
        // Whether to reattempt the kick if we lose ball control 
        bool retry_kick = true;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the DribbleSkillFSM to get control of the ball and pivot
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void getBallControlAndPivot(
        const Update& event,
        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Action that kicks the ball
     *
     * @param event the Update event
     */
    void kickBall(const Update& event);

    /**
     * Guard that checks if the robot has lost control of the ball
     *
     * @param event the Update event
     *
     * @return if the robot has lost control of the ball
     */
    bool lostBallControl(const Update& event);

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event the Update event
     *
     * @return if the ball has been kicked
     */
    bool ballKicked(const Update& event);

    /**
     * Guard that checks if reattempting a failed kick is allowed
     *
     * @param event the Update event
     *
     * @return whether kick retries are allowed 
     */
    bool retryKickAllowed(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_STATE(DribbleSkillFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(lostBallControl)
        DEFINE_SML_GUARD(ballKicked)
        DEFINE_SML_GUARD(retryKickAllowed)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControlAndPivot, DribbleSkillFSM)
        DEFINE_SML_ACTION(kickBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E / getBallControlAndPivot_A = DribbleSkillFSM_S,

            DribbleSkillFSM_S + Update_E / getBallControlAndPivot_A,
            DribbleSkillFSM_S = KickState_S,

            KickState_S + Update_E[ballKicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            KickState_S + Update_E[lostBallControl_G && retryKickAllowed_G] 
                / getBallControlAndPivot_A = DribbleSkillFSM_S,
            KickState_S + Update_E[lostBallControl_G && !retryKickAllowed_G] 
                / SET_STOP_PRIMITIVE_ACTION = X,
            KickState_S + Update_E / kickBall_A,

            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }
};
