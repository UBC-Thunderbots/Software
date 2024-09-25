#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"

struct MoveSkillFSM
{
   public:
    class MoveState;

    struct ControlParams
    {
        // The point the robot is trying to move to
        Point destination;
        // The orientation the robot should have when it arrives at its destination
        Angle final_orientation;
        // The speed the robot should have when it arrives at its destination
        double final_speed;
        // How to run the dribbler
        TbotsProto::DribblerMode dribbler_mode = TbotsProto::DribblerMode::OFF;
        // How to navigate around the ball
        TbotsProto::BallCollisionType ball_collision_type =
            TbotsProto::BallCollisionType::AVOID;
        // The command to autochip or autokick
        AutoChipOrKick auto_chip_or_kick = {AutoChipOrKickMode::OFF, 0};
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT;
        // The obstacle avoidance mode
        TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode =
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE;
        // The target spin while moving in revolutions per second
        double target_spin_rev_per_s = 0.0;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Action that sets the primitive to a move primitive corresponding
     * to the given control params
     *
     * @param event MoveSkillFSM::Update event
     */
    void updateMove(const Update &event);

    /**
     * Guard that checks if the robot is done moving
     *
     * @param event MoveSkillFSM::Update event
     *
     * @return if robot has reached the destination
     */
    bool moveDone(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(moveDone)
        DEFINE_SML_ACTION(updateMove)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveState_S + Update_E[!moveDone_G] / updateMove_A = MoveState_S,
            MoveState_S + Update_E[moveDone_G] / updateMove_A   = X,
            X + Update_E[!moveDone_G] / updateMove_A            = MoveState_S,
            X + Update_E / updateMove_A                         = X);
    }
};
