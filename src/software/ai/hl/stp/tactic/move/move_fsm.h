#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

struct MoveFSM
{
   public:
    // these classes define the states used in the transition table
    // they are exposed so that tests can check if the FSM is in a particular state
    class MoveState;

    // this struct defines the unique control parameters that the MoveFSM requires in its
    // update
    struct ControlParams
    {
        // The point the robot is trying to move to
        Point destination;
        // The orientation the robot should have when it arrives at its destination
        Angle final_orientation;
        // The speed the robot should have when it arrives at its destination
        double final_speed;
        // How to run the dribbler
        TbotsProto::DribblerMode dribbler_mode;
        // How to navigate around the ball
        TbotsProto::BallCollisionType ball_collision_type;
        // The command to autochip or autokick
        AutoChipOrKick auto_chip_or_kick;
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
        // The target spin while moving in revolutions per second
        double target_spin_rev_per_s;
    };

    // this struct defines the only event that the MoveFSM responds to
    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * This is an Action that sets the primitive to a move primitive corresponding to the
     * Update_E event
     *
     * @param event MoveFSM::Update event
     */
    void updateMove(const Update &event);

    /**
     * This is an Action that sets the primitive to a stop primitive corresponding to the
     * Update_E event
     *
     * @param event MoveFSM::Update event
     */
    void stop(const Update &event);

    /**
     * This guard is used to check if the robot is done moving
     *
     * @param event MoveFSM::Update event
     *
     * @return if robot has reached the destination
     */
    bool moveDone(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        // MoveState_S is the _state_ used in the transition table
        DEFINE_SML_STATE(MoveState)
        // Update_E is the _event_ that the MoveFSM responds to
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(moveDone)
        DEFINE_SML_ACTION(updateMove)
        DEFINE_SML_ACTION(stop)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveState_S + Update_E[!moveDone_G] / updateMove_A = MoveState_S,
            MoveState_S + Update_E[moveDone_G] / updateMove_A   = X,
            X + Update_E[!moveDone_G] / updateMove_A            = MoveState_S,
X + Update_E / stop_A            = X
            );
    }
};
