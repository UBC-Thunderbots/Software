#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

struct KickFSM
{
   public:
    class KickState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
    };


    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that updates the MovePrimitive
     *
     * @param event KickFSM::Update event
     */
    void updateKick(const Update &event);

    /**
     * Action that updates the GetBehindBallFSM
     *
     * @param event KickFSM::Update event
     * @param processEvent processes the GetBehindBallFSM::Update
     */
    void updateGetBehindBall(
        const Update &event,
        boost::sml::back::process<GetBehindBallFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been chicked
     *
     * @param event KickFSM::Update event
     *
     * @return if the ball has been chicked
     */
    bool ballChicked(const Update &event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallFSM)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballChicked)
        DEFINE_SML_ACTION(updateKick)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateGetBehindBall, GetBehindBallFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallFSM_S + Update_E / updateGetBehindBall_A,
            GetBehindBallFSM_S                                    = KickState_S,
            KickState_S + Update_E[!ballChicked_G] / updateKick_A = KickState_S,
            KickState_S + Update_E[ballChicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                          = X);
    }

   private:
};
