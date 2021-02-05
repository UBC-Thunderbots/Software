#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct InterceptBallFSM
{
    // these classes define the states used in the transition table
    // they are exposed so that tests can check if the FSM is in a particular state
    class idle_state;
    class block_ball_state;
    class wait_for_ball_state;
    class slow_intercept_1_state;
    class slow_intercept_2_state;

    // this struct defines the unique control parameters that the InterceptBallFSM
    // requires in its update
    struct ControlParams
    {
    };

    // this struct defines the only event that the InterceptBallFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    Point fastInterceptionPoint(const Robot &robot, const Ball &ball, const Field &field)
    {
        static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
        Point intercept_position                                   = ball.position();
        while (contains(field.fieldLines(), intercept_position))
        {
            Duration ball_time_to_position = Duration::fromSeconds(
                distance(intercept_position, ball.position()) / ball.velocity().length());
            Duration robot_time_to_pos = getTimeToPositionForRobot(
                robot.position(), intercept_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

            if (robot_time_to_pos < ball_time_to_position)
            {
                break;
            }
            intercept_position +=
                ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
        }
        return intercept_position;
    }

    auto operator()()
    {
        using namespace boost::sml;

        // idle_s and move_s are the two _states_ used in the transition table
        const auto idle_s             = state<idle_state>;
        const auto block_ball_s       = state<block_ball_state>;
        const auto wait_for_ball_s    = state<wait_for_ball_state>;
        const auto slow_intercept_1_s = state<slow_intercept_1_state>;
        //        const auto slow_intercept_2_s = state<slow_intercept_2_state>;

        // update_e is the _event_ that the InterceptBallFSM responds to
        const auto update_e = event<Update>;

        const auto block_ball = [this](auto event) {
            Point intercept_position =
                fastInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), intercept_position,
                (-event.common.world.ball().velocity()).orientation(), 0,
                DribblerMode::MAX_FORCE, BallCollisionType::AVOID));
        };

        const auto wait_for_ball = [this](auto event) {
            Point intercept_position =
                fastInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), intercept_position,
                (-event.common.world.ball().velocity()).orientation(), 0,
                DribblerMode::MAX_FORCE, BallCollisionType::AVOID));
        };

        const auto ball_moving_slow = [](auto event) {
            static const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;
            return event.common.world.ball().velocity().length() <
                   BALL_MOVING_SLOW_SPEED_THRESHOLD;
        };

        const auto robot_in_position = [this](auto event) {
            static const double ROBOT_STOPPED_SPEED_M_PER_S = 0.03;
            Point intercept_position =
                fastInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            auto ball = event.common.world.ball();
            bool intercept_point_in_front_of_ball =
                acuteAngle(ball.velocity(), intercept_position - ball.position()) <
                Angle::quarter();
            bool robot_at_intercept_point =
                distance(event.common.robot.position(), intercept_position) < 0.05;
            bool robot_stopped =
                event.common.robot.velocity().length() < ROBOT_STOPPED_SPEED_M_PER_S;
            return intercept_point_in_front_of_ball && robot_at_intercept_point &&
                   robot_stopped;
        };

        const auto intercepted = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e[!ball_moving_slow] / block_ball = block_ball_s,
            *idle_s + update_e[ball_moving_slow]               = slow_intercept_1_s,
            block_ball_s + update_e[!robot_in_position] / block_ball,
            block_ball_s + update_e[robot_in_position] / wait_for_ball = wait_for_ball_s,
            wait_for_ball_s + update_e[intercepted]                    = X,
            wait_for_ball_s + update_e[!intercepted] / wait_for_ball,
            slow_intercept_1_s = X
            // move_s + update_e[!move_done] / update_move = move_s,
            // move_s + update_e[move_done] / update_move  = X,
            // X + update_e[move_done] / update_move       = X
        );
    }
};
