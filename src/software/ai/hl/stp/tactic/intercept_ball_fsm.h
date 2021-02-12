#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/move_fsm.h"
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
    class wait_for_ball_state;
    class chase_ball_state;
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

        const auto idle_s          = state<idle_state>;
        const auto wait_for_ball_s = state<wait_for_ball_state>;
        const auto chase_ball_s    = state<chase_ball_state>;
        const auto move_s          = state<MoveFSM>;

        // update_e is the _event_ that the InterceptBallFSM responds to
        const auto update_e = event<Update>;

        const auto block_ball = [this](auto event,
                                       back::process<MoveFSM::Update> processEvent) {
            Point intercept_position =
                fastInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            MoveFSM::ControlParams control_params{
                .destination = intercept_position,
                .final_orientation =
                    (-event.common.world.ball().velocity()).orientation(),
                .final_speed = 0};

            processEvent(MoveFSM::Update(control_params, event.common));
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

        const auto chase_ball = [this](auto event) {
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), ball_position, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW));
        };

        const auto ball_moving_slow = [](auto event) {
            static const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;
            return event.common.world.ball().velocity().length() <
                   BALL_MOVING_SLOW_SPEED_THRESHOLD;
        };

        const auto intercepted = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e[!ball_moving_slow] / block_ball = move_s,
            *idle_s + update_e[ball_moving_slow] / chase_ball  = chase_ball_s,
            move_s + update_e / block_ball,
            move_s = wait_for_ball_s,  // wait for ball when in position
            wait_for_ball_s + update_e[!intercepted] / wait_for_ball,
            wait_for_ball_s + update_e[intercepted] = X,
            chase_ball_s + update_e[!intercepted && ball_moving_slow] / chase_ball,
            chase_ball_s + update_e[intercepted] = X);
    }
};
