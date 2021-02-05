#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

struct InterceptBallFSM
{
    // these classes define the states used in the transition table
    // they are exposed so that tests can check if the FSM is in a particular state
    class idle_state;
    class fast_intercept_1_state;
    class fast_intercept_2_state;
    class slow_intercept_1_state;
    class slow_intercept_2_state;

    // this struct defines the unique control parameters that the InterceptBallFSM
    // requires in its update
    struct ControlParams
    {
    };

    // this struct defines the only event that the InterceptBallFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        // idle_s and move_s are the two _states_ used in the transition table
        const auto idle_s             = state<idle_state>;
        const auto fast_intercept_1_s = state<fast_intercept_1_state>;
        //        const auto fast_intercept_2_s = state<fast_intercept_2_state>;
        const auto slow_intercept_1_s = state<slow_intercept_1_state>;
        //        const auto slow_intercept_2_s = state<slow_intercept_2_state>;

        // update_e is the _event_ that the InterceptBallFSM responds to
        const auto update_e = event<Update>;

        // this action sets the intent to a move intent corresponding to the update_e
        // event
        //        const auto update_move = [](auto event) {
        //            event.common.set_intent(std::make_unique<MoveIntent>(
        //                event.common.robot.id(), event.common.world.ball().position(),
        //                Angle::zero(), 0, DribblerMode::OFF, BallCollisionType::AVOID));
        //        };

        const auto ball_moving_slow = [](auto event) {
            static const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;
            return event.common.world.ball().velocity().length() <
                   BALL_MOVING_SLOW_SPEED_THRESHOLD;
        };

        // this guard is used check if the robot is done moving
        //        const auto move_done = [](auto event) {
        //            return robotReachedDestination(event.common.robot,
        //                                           event.control_params.destination,
        //                                           event.control_params.final_orientation);
        //        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e[!ball_moving_slow] = fast_intercept_1_s,
            *idle_s + update_e[ball_moving_slow]  = slow_intercept_1_s,
            fast_intercept_1_s = X, slow_intercept_1_s = X
            // move_s + update_e[!move_done] / update_move = move_s,
            // move_s + update_e[move_done] / update_move  = X,
            // X + update_e[move_done] / update_move       = X
        );
    }
};
