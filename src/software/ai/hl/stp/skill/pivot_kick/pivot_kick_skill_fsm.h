#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct PivotKickSkillFSM
{
    class KickStartState;
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
     * Action that sets the kick start time
     *
     * @param event the Update event
     */
    void setKickStartTime(const Update& event);

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

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_STATE(KickStartState)
        DEFINE_SML_STATE(DribbleSkillFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(lostBallControl)
        DEFINE_SML_GUARD(ballKicked)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControlAndPivot, DribbleSkillFSM)
        DEFINE_SML_ACTION(setKickStartTime)
        DEFINE_SML_ACTION(kickBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E / getBallControlAndPivot_A = DribbleSkillFSM_S,

            DribbleSkillFSM_S + Update_E / getBallControlAndPivot_A,
            DribbleSkillFSM_S = KickStartState_S,

            KickStartState_S + Update_E / (setKickStartTime_A, kickBall_A) = KickState_S,

            KickState_S + Update_E[ballKicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            KickState_S + Update_E[lostBallControl_G] / getBallControlAndPivot_A =
                DribbleSkillFSM_S,
            KickState_S + Update_E / kickBall_A,

            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    Timestamp kick_start_time_;
};
