#include "software/ai/hl/stp/action/intercept_ball_action.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

InterceptBallAction::InterceptBallAction(const Field& field, const Ball& ball)
    : Action(false), field(field), ball(ball)
{
}

void InterceptBallAction::updateWorldParams(const World& world)
{
    this->ball  = world.ball();
    this->field = world.field();
}

void InterceptBallAction::updateControlParams(const Robot& robot)
{
    this->robot = robot;
}

void InterceptBallAction::interceptSlowBall(IntentCoroutine::push_type& yield)
{
    // Run into the ball until the ball is in the dribbler and the ball's velocity
    // is similar to the robot's (so we know we have control of the ball)
    bool intercept_done = false;
    do
    {
        auto face_ball_orientation = (ball.position() - robot->position()).orientation();
        yield(std::make_unique<MoveIntent>(
            robot->id(), ball.position(), face_ball_orientation, 0,
            DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot->robotConstants()));

        // Restart the action if the ball's speed has sped up substantially.
        // The extra factor of 2 is to prevent being overly sensitive to restarts
        // if the ball's speed is right at the threshold
        if (ball.velocity().length() > BALL_MOVING_SLOW_SPEED_THRESHOLD * 2)
        {
            restart();
        }

        bool ball_very_close_to_robot_dribbler =
            distance(ball.position(),
                     robot->position() + Vector::createFromAngle(robot->orientation())
                                             .normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) <
            BALL_CLOSE_TO_DRIBBLER_THRESHOLD;
        Vector dribbler_tangential_velocity =
            Vector::createFromAngle(robot->orientation())
                .perpendicular()
                .normalize((DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS) *
                           robot->angularVelocity().toRadians());
        Vector expected_ball_velocity = robot->velocity() + dribbler_tangential_velocity;
        bool velocity_magnitudes_similar =
            std::fabs(expected_ball_velocity.length() - ball.velocity().length()) <
            SIMILAR_VELOCITY_MAGNITUDE_THRESHOLD;
        bool velocity_angles_similar =
            expected_ball_velocity.orientation().minDiff(ball.velocity().orientation()) <
            SIMILAR_VELOCITY_ANGLE_THRESHOLD;
        bool ball_velocity_similar_to_robot =
            velocity_angles_similar && velocity_magnitudes_similar;
        intercept_done =
            ball_very_close_to_robot_dribbler && ball_velocity_similar_to_robot;
    } while (!intercept_done);

    // Stop while keeping the ball in the dribbler
    do
    {
        auto face_ball_orientation = (ball.position() - robot->position()).orientation();

        yield(std::make_unique<MoveIntent>(
            robot->id(), robot->position(), face_ball_orientation, 0,
            DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot->robotConstants()));
    } while (robot->velocity().length() > ROBOT_STOPPED_SPEED_M_PER_S);
}

void InterceptBallAction::interceptFastBall(IntentCoroutine::push_type& yield)
{
    // Find the first point where the robot can get to before the ball
    Point intercept_position = ball.position();
    while (contains(field.fieldLines(), intercept_position))
    {
        Duration ball_time_to_position = Duration::fromSeconds(
            distance(intercept_position, ball.position()) / ball.velocity().length());
        Duration robot_time_to_pos = getTimeToPositionForRobot(
            robot->position(), intercept_position,
            robot->robotConstants().robot_max_speed_meters_per_second,
            robot->robotConstants().robot_max_acceleration_meters_per_second_squared);

        if (robot_time_to_pos < ball_time_to_position)
        {
            break;
        }
        intercept_position +=
            ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
    }

    // Move to the intercept position and stop while avoiding the ball
    bool robot_in_intercept_position = false;
    do
    {
        yield(std::make_unique<MoveIntent>(
            robot->id(), intercept_position, (-ball.velocity()).orientation(), 0,
            DribblerMode::MAX_FORCE, BallCollisionType::AVOID,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot->robotConstants()));

        // Restart the action if the ball's speed has slowed down substantially.
        // The extra factor of 2 is to prevent being overly sensitive to restarts
        // if the ball's speed is right at the threshold
        if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD / 2.0)
        {
            restart();
        }

        bool intercept_point_in_front_of_ball =
            acuteAngle(ball.velocity(), intercept_position - ball.position()) <
            Angle::quarter();
        bool robot_at_intercept_point = distance(robot->position(), intercept_position) <
                                        MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD;
        bool robot_stopped = robot->velocity().length() < ROBOT_STOPPED_SPEED_M_PER_S;
        robot_in_intercept_position =
            intercept_point_in_front_of_ball && robot_at_intercept_point && robot_stopped;
    } while (!robot_in_intercept_position);

    // Wait for the ball to arrive at the robot
    bool intercept_done = false;
    do
    {
        yield(std::make_unique<MoveIntent>(
            robot->id(), intercept_position, (-ball.velocity()).orientation(), 0,
            DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot->robotConstants()));

        if (!intercept_done &&
            ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD / 2.0)
        {
            restart();
        }

        bool ball_very_close_to_robot_dribbler =
            distance(ball.position(),
                     robot->position() + Vector::createFromAngle(robot->orientation())
                                             .normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) <
            BALL_CLOSE_TO_DRIBBLER_THRESHOLD;
        intercept_done = ball_very_close_to_robot_dribbler;
    } while (!intercept_done);
}

void InterceptBallAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        interceptSlowBall(yield);
    }
    else
    {
        interceptFastBall(yield);
    }
}
