#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

struct MoveFSM
{
    // these classes define the states used in the transition table
    // they are exposed so that tests can check if the FSM is in a particular state
    class idle_state;
    class move_state;

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
    };

    // this struct defines the only event that the MoveFSM responds to
    UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        // idle_s and move_s are the two _states_ used in the transition table
        const auto idle_s = state<idle_state>;
        const auto move_s = state<move_state>;

        // update_e is the _event_ that the MoveFSM responds to
        const auto update_e = event<Update>;

        // this action sets the intent to a move intent corresponding to the update_e
        // event
        const auto update_move = [](auto event) {
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), event.control_params.destination,
                event.control_params.final_orientation, event.control_params.final_speed,
                DribblerMode::OFF, BallCollisionType::AVOID));
        };

        // this guard is used check if the robot is done moving
        const auto move_done = [](auto event) {
            return robotReachedDestination(event.common.robot,
                                           event.control_params.destination,
                                           event.control_params.final_orientation);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e / update_move            = move_s,
            move_s + update_e[!move_done] / update_move = move_s,
            move_s + update_e[move_done] / update_move  = X,
            X + update_e[move_done] / update_move       = X);
    }
};
