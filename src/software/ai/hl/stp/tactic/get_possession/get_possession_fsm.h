#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct GetPossessionFSM
{
    class GetPossessionState;

    struct ControlParams
    {
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Calculates the interception point for intercepting balls
     *
     * @param robot The robot to do the interception
     * @param ball The ball to intercept
     * @field The field to intercept on
     *
     * @return the best interception point
     */
    // TODO (#1968): Merge this functionality with findBestInterceptForBall in the
    // evaluation folder
    static Point findInterceptionPoint(const Robot &robot, const Ball &ball,
                                       const Field &field)
    {
        static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
        static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
        if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
        {
            auto face_ball_vector = (ball.position() - robot.position());
            auto point_in_front_of_ball =
                ball.position() -
                face_ball_vector.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                           BALL_MAX_RADIUS_METERS);
            return point_in_front_of_ball;
        }
        Point intercept_position = ball.position();
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

        const auto get_possession_s = state<GetPossessionState>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the ball has been have_possession
         *
         * @param event GetPossessionFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto have_possession = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * Action to update the MoveIntent to hold onto the ball
         *
         * @param event GetPossessionFSM::Update
         */
        const auto hold_ball = [this](auto event) {
            auto ball_position    = event.common.world.ball().position();
            auto face_ball_vector = (ball_position - event.common.robot.position());
            auto point_in_front_of_ball =
                ball_position - face_ball_vector.normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                                           BALL_MAX_RADIUS_METERS);
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), point_in_front_of_ball,
                face_ball_vector.orientation(), 0, DribblerMode::MAX_FORCE,
                BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        /**
         * Action to update to get possession of the ball
         *
         * If the ball is moving quickly, then move in front of the ball
         * If the ball is moving slowly, then chase the ball
         *
         * @param event GetPossessionFSM::Update
         */
        const auto get_possession = [this](auto event) {
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();
            Point intercept_position =
                findInterceptionPoint(event.common.robot, event.common.world.ball(),
                                      event.common.world.field());
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), intercept_position, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *get_possession_s + update_e[have_possession] / get_possession = X,
            get_possession_s + update_e[!have_possession] / get_possession,
            X + update_e[!have_possession] / get_possession = get_possession_s,
            X + update_e[have_possession] / hold_ball);
    }
};
