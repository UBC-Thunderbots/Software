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

    // ASCII art showing the region behind the ball
    // Diagram not to scale
    //
    //                 X
    //          v---------------------v
    //
    //       >  B---------------------C
    //       |   \                   /
    //       |    \                 /
    //       |     \               /     <- Region considered "behind chick origin"
    //     X |      \             /
    //       |       \           /
    //       |        \         /
    //                A1---A---A2   <  The chick origin is at A
    //                     |
    //                     V
    //             direction of chip/kick

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

   private:
    // How large the triangle is that defines the region where the robot is
    // behind the chick origin and ready to chip or kick.
    // We want to keep the region small enough that we won't use the
    // Chip/KickPrimitive from too far away (since the Chip/KickPrimitive doesn't avoid
    // obstacles and we risk colliding with something), but large enough we can
    // reasonably get in the region and chip/kick the ball successfully. This
    // value is 'X' in the ASCII art below
    double size_of_region_behind_ball;
};
