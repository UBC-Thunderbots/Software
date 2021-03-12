#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

struct MoveFSM
{
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
        DribblerMode dribbler_mode;
        // How to navigate around the ball
        BallCollisionType ball_collision_type;
        // The command to autochip or autokick
        AutoChipOrKick auto_chip_or_kick;
        // The maximum allowed speed mode
        MaxAllowedSpeedMode max_allowed_speed_mode;
    };

    // this struct defines the only event that the MoveFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        // move_s is the _state_ used in the transition table
        const auto move_s = state<MoveState>;

        // update_e is the _event_ that the MoveFSM responds to
        const auto update_e = event<Update>;

        /**
         * This is an Action that sets the intent to a move intent corresponding to the
         * update_e event
         *
         * @param event MoveFSM::Update event
         */
        const auto update_move = [](auto event) {
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), event.control_params.destination,
                event.control_params.final_orientation, event.control_params.final_speed,
                event.control_params.dribbler_mode,
                event.control_params.ball_collision_type,
                event.control_params.auto_chip_or_kick,
                event.control_params.max_allowed_speed_mode));
        };

        /**
         * This guard is used to check if the robot is done moving
         *
         * @param event MoveFSM::Update event
         *
         * @return if robot has reached the destination
         */
        const auto move_done = [](auto event) {
            return robotReachedDestination(event.common.robot,
                                           event.control_params.destination,
                                           event.control_params.final_orientation);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *move_s + update_e[!move_done] / update_move = move_s,
            move_s + update_e[move_done] / update_move   = X,
            X + update_e[!move_done] / update_move       = move_s);
    }
};
