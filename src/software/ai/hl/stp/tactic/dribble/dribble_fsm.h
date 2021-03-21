#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct DribbleFSM
{
    class GetPossessionState;
    class DribbleState;

    struct ControlParams
    {
        std::optional<Point> ball_destination;
        std::optional<Angle> final_face_ball_oriention;
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
            auto face_ball_vector       = (ball.position() - robot.position());
            auto point_in_front_of_ball = convertBallPositionToRobotPosition(
                ball.position(), face_ball_vector.orientation());
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

    static Point convertBallPositionToRobotPosition(const Point &ball_position,
                                                    const Angle &face_ball_angle)
    {
        return ball_position -
               Vector::createFromAngle(face_ball_angle)
                   .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);
    }

    /**
     * Gets the ball destination from the update event
     *
     * @param event DribbleBallFSM::Update
     *
     * @return the final destination to dribble the ball to
     */
    static Point getBallDestination(DribbleFSM::Update event)
    {
        auto ball_position = event.common.world.ball().position();
        // Default is the current ball position
        Point target_dest = ball_position;
        if (event.control_params.ball_destination)
        {
            target_dest = event.control_params.ball_destination.value();
        }
        return target_dest;
    }

    static Angle getFinalFaceBallOrientation(DribbleFSM::Update event)
    {
        // Default is face ball direction
        Angle target_orientation =
            (event.common.world.ball().position() - event.common.robot.position())
                .orientation();
        if (event.control_params.final_face_ball_oriention)
        {
            target_orientation = event.control_params.final_face_ball_oriention.value();
        }
        return target_orientation;
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_possession_s = state<GetPossessionState>;
        const auto dribble_s        = state<DribbleState>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the ball has been have_possession
         *
         * @param event DribbleFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto have_possession = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * Guard that checks if the ball is at the ball_destination
         *
         * @param event DribbleBallFSM::Update
         *
         * @return if the ball is at the ball_destination
         */
        const auto ball_at_dest = [](auto event) {
            return ((event.common.world.ball().position() - getBallDestination(event))
                        .length() < 0.05);
        };

        /**
         * Action to update to get possession of the ball
         *
         * If the ball is moving quickly, then move in front of the ball
         * If the ball is moving slowly, then chase the ball
         *
         * @param event DribbleFSM::Update
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
                MaxAllowedSpeedMode::PHYSICAL_LIMIT));
        };

        /**
         * Action to update to get possession of the ball
         *
         * @param event DribbleBallFSM::Update
         */
        const auto dribble = [this, ball_at_dest](auto event) {
            auto ball_position    = event.common.world.ball().position();
            auto ball_destination = getBallDestination(event);
            Angle to_destination_orientation =
                (ball_destination - ball_position).orientation();
            auto final_face_ball_oriention = getFinalFaceBallOrientation(event);
            if (ball_at_dest(event))
            {
                // pivot to final face ball destination
                auto target_destination = convertBallPositionToRobotPosition(
                    ball_destination, final_face_ball_oriention);
                event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), target_destination,
                    final_face_ball_oriention, 0, DribblerMode::MAX_FORCE,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT));
            }
            else if (to_destination_orientation.minDiff(
                         event.common.robot.orientation()) < Angle::fromDegrees(5))
            {
                // dribble towards ball destination
                event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(),
                    convertBallPositionToRobotPosition(ball_destination,
                                                       to_destination_orientation),
                    to_destination_orientation, 0, DribblerMode::MAX_FORCE,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT));
            }
            else
            {
                // pivot to face ball destination
                event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(),
                    convertBallPositionToRobotPosition(ball_position,
                                                       to_destination_orientation),
                    to_destination_orientation, 0, DribblerMode::MAX_FORCE,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT));
            }
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *get_possession_s + update_e[have_possession] / dribble = dribble_s,
            get_possession_s + update_e[!have_possession] / get_possession,
            dribble_s + update_e[!have_possession] / get_possession = get_possession_s,
            dribble_s + update_e[!ball_at_dest] / dribble,
            dribble_s + update_e[ball_at_dest] / dribble    = X,
            X + update_e[!have_possession] / get_possession = get_possession_s,
            X + update_e[!ball_at_dest] / dribble = dribble_s, X + update_e / dribble);
    }
};
