#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/triangle.h"

struct GetBehindBallSkillFSM
{
   public:
    class GetBehindBallState;

    struct ControlParams
    {
        // The location where the chick will be taken, i.e. where we expect the ball to be
        // when we chip or kick it
        Point ball_location;
        // The direction the Robot will chick in
        Angle chick_direction;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    GetBehindBallSkillFSM();

    /**
     * Action that updates the MovePrimitive
     *
     * @param event GetBehindBallSkillFSM::Update event
     */
    void updateMove(const Update& event);

    /**
     * Guard that checks if the robot is behind the ball
     *
     * @param event GetBehindBallSkillFSM::Update event
     *
     * @return if the robot is behind the ball
     */
    bool behindBall(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBehindBallState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(behindBall)
        DEFINE_SML_ACTION(updateMove)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBehindBallState_S + Update_E[!behindBall_G] / updateMove_A,
            GetBehindBallState_S + Update_E[behindBall_G] / updateMove_A = X,
            X + Update_E[!behindBall_G] / updateMove_A = GetBehindBallState_S,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION   = X);
    }
};
