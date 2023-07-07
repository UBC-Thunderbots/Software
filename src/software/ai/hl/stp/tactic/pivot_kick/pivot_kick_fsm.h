#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"

struct PivotKickFSM
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

        std::optional<Angle> dribble_into_ball_orientation = std::nullopt;
    };

    // this struct defines the only event that the PivotKickFSM responds to
    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the DribbleFSM to get possession of the ball and pivot
     *
     * @param event PivotKickFSM::Update event
     * @param processEvent processes the GetBehindBallFSM::Update
     */
    void getPossessionAndPivot(
        const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * Action that kicks the ball
     *
     * @param event PivotKickFSM::Update event
     */
    void kickBall(const Update& event);

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event PivotKickFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool ballKicked(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(StartState)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_STATE(DribbleFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballKicked)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getPossessionAndPivot, DribbleFSM)
        DEFINE_SML_ACTION(kickBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *StartState_S + Update_E / getPossessionAndPivot_A = DribbleFSM_S,
            DribbleFSM_S + Update_E / getPossessionAndPivot_A, DribbleFSM_S = KickState_S,
            KickState_S + Update_E[!ballKicked_G] / kickBall_A,
            KickState_S + Update_E[ballKicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                         = X);
    }
};
