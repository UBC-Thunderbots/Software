#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct FastBallInterceptFSM
{
    class wait_for_ball;

    struct ControlParams
    {
    };

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

        const auto move_s          = state<MoveFSM>;
        const auto wait_for_ball_s = state<wait_for_ball>;

        // update_e is the _event_ that the InterceptBallFSM responds to
        const auto update_e = event<Update>;

        /**
         * Action to update MoveFSM
         *
         * @param event FastBallInterceptFSM::Update event
         * @param processEvent processes the Move::Update
         */
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

        /**
         * Action to update MoveIntent to wait for the ball
         *
         * @param event FastBallInterceptFSM::Update event
         */
        const auto wait_for_ball = [this](auto event) {
            Point intercept_position =
                fastInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), intercept_position,
                (-event.common.world.ball().velocity()).orientation(), 0,
                DribblerMode::MAX_FORCE, BallCollisionType::AVOID));
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *move_s + update_e / block_ball,
            move_s = wait_for_ball_s,  // wait for ball when in position
            wait_for_ball_s + update_e / wait_for_ball);
    }
};

struct InterceptBallFSM
{
    class chase_ball;

    struct ControlParams
    {
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto chase_ball_s     = state<chase_ball>;
        const auto fast_intercept_s = state<FastBallInterceptFSM>;

        // update_e is the _event_ that the InterceptBallFSM responds to
        const auto update_e = event<Update>;

        /**
         * Guard that checks if the ball is slow
         *
         * @param event InterceptBallFSM::Update
         *
         * @return if the ball is slow
         */
        const auto slow_ball = [](auto event) {
            static const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;
            return event.common.world.ball().velocity().length() <
                   BALL_MOVING_SLOW_SPEED_THRESHOLD;
        };

        /**
         * Guard that checks if the ball has been intercepted
         *
         * @param event InterceptBallFSM::Update
         *
         * @return if the ball has been intercepted
         */
        const auto intercepted = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * Action to update the MoveIntent to chase the ball
         *
         * @param event InterceptBallFSM::Update
         */
        const auto chase_ball = [this](auto event) {
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), ball_position, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW));
        };

        /**
         * Action to update the FastBallInterceptFSM
         *
         * @param event FastBallInterceptFSM::Update event
         * @param processEvent processes the FastBallInterceptFSM::Update
         */
        const auto fast_intercept =
            [](auto event, back::process<FastBallInterceptFSM::Update> processEvent) {
                processEvent(FastBallInterceptFSM::Update({}, event.common));
            };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *chase_ball_s + update_e[intercepted] / chase_ball = X,
            chase_ball_s + update_e[slow_ball] / chase_ball,
            chase_ball_s + update_e[!slow_ball] / fast_intercept  = fast_intercept_s,
            fast_intercept_s + update_e[intercepted] / chase_ball = X,
            fast_intercept_s + update_e[!slow_ball] / fast_intercept,
            fast_intercept_s + update_e[slow_ball] / chase_ball = chase_ball_s,
            X + update_e[!intercepted] / chase_ball             = chase_ball_s,
            X + update_e[intercepted] / chase_ball);
    }
};
