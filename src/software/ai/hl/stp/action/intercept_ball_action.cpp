#include "software/ai/hl/stp/action/intercept_ball_action.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/util.h"
#include "software/logger/logger.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/util/acute_angle.h"
#include "software/new_geom/util/closest_point.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"

InterceptBallAction::InterceptBallAction(const Field& field, const Ball& ball,
                                         bool loop_forever)
    : Action(loop_forever), field(field), ball(ball)
{
}

void InterceptBallAction::updateWorldParams(const Field& field, const Ball& ball)
{
    this->ball  = ball;
    this->field = field;
}

void InterceptBallAction::updateControlParams(const Robot& robot)
{
    this->robot = robot;
}

void InterceptBallAction::accept(MutableActionVisitor& visitor)
{
    visitor.visit(*this);
}

std::optional<Point> InterceptBallAction::getPointBallLeavesField(const Field& field,
                                                                  const Ball& ball)
{
    Ray ball_ray(ball.position(), ball.velocity());
    if (field.pointInFieldLines(ball.position()))
    {
        std::unordered_set<Point> intersections =
            intersection(field.fieldLines(), ball_ray);
        if (!intersections.empty())
        {
            return *intersections.begin();
        }
    }

    return std::nullopt;
}

void InterceptBallAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // This action tries to intercept and collect the ball on the field.
    // We find the point on the Ray formed by the ball's velocity that is closest to
    // the robot. If the time for the robot to get to that point is less than the time
    // for the ball to get to that point, the robot moves to that point to intercept the
    // ball. Once the robot is in the ball's path, it will move to a point just in
    // front of the ball in order to meet it quicker, while staying in it's path.
    //
    // If the robot cannot reach the closest point on the ray before the ball, the robot
    // will move to intercept the ball at the point it would leave the field. This
    // generally causes the robot to move far enough ahead of the ball that it can switch
    // to intercepting at the closest point on the ray like above.
    //
    // Finally, if the ball is moving slowly the robot will go directly to the ball.
    do
    {
        Point closest_point = closestPointOnLine(
            robot->position(), Line(ball.position(), ball.position() + ball.velocity()));
        bool point_in_front_of_ball =
            acuteAngle(ball.velocity(), closest_point - ball.position()) <
            Angle::quarter();

        // We add 1e-6 to avoid division by 0 without affecting the result significantly
        Duration ball_time_to_position = Duration::fromSeconds(
            distance(closest_point, ball.position()) / (ball.velocity().length() + 1e-6));
        Duration robot_time_to_pos = getTimeToPositionForRobot(
            robot->position(), closest_point, ROBOT_MAX_SPEED_METERS_PER_SECOND,
            ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        std::optional<Point> intercept_pos = std::nullopt;
        if (point_in_front_of_ball && (ball_time_to_position > robot_time_to_pos))
        {
            intercept_pos = closest_point;
        }

        auto point_ball_leaves_field = getPointBallLeavesField(field, ball);
        if (intercept_pos)
        {
            while (point_in_front_of_ball)
            {
                moveToInterceptPosition(yield, closest_point);

                closest_point = closestPointOnLine(
                    robot->position(),
                    Line(ball.position(), ball.position() + ball.velocity()));
                point_in_front_of_ball =
                    acuteAngle(ball.velocity(), closest_point - ball.position()) <
                    Angle::quarter();
            }
        }
        else if (point_ball_leaves_field)
        {
            LOG(DEBUG) << "ball leaving field" << std::endl;
            yield(std::make_unique<MoveIntent>(
                robot->id(), point_ball_leaves_field.value(),
                (ball.position() - robot->position()).orientation(), 0, 0,
                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
                BallCollisionType::ALLOW));
        }
        else
        {
            // This is a fallback case that ideally should never be reached. We will only
            // enter this case if the robot is not in front of the ball, and the ball is
            // not within the field.
            LOG(DEBUG) << "Moving to ball backup case" << std::endl;
            yield(std::make_unique<MoveIntent>(
                robot->id(), ball.position(),
                (ball.position() - robot->position()).orientation(), 0, 0,
                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
                BallCollisionType::ALLOW));
        }
    } while (!robotHasPossession(ball.getPreviousStates(), robot->getPreviousStates()));
}

void InterceptBallAction::moveToInterceptPosition(IntentCoroutine::push_type& yield,
                                                  Point closest_point_on_ball_trajectory)
{
    bool robot_on_ball_line =
        acuteAngle(ball.velocity(), robot->position() - ball.position()) <
            Angle::quarter() &&
        distance(robot->position(),
                 Line(ball.position(), ball.position() + ball.velocity())) <
            ROBOT_CLOSE_TO_BALL_TRAJECTORY_LINE_THRESHOLD;

    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        LOG(DEBUG) << "moving to ball slow" << std::endl;
        yield(std::make_unique<MoveIntent>(
            robot->id(), ball.position(),
            (ball.position() - robot->position()).orientation(), FINAL_SPEED_AT_SLOW_BALL,
            0, DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
            BallCollisionType::ALLOW));
    }
    else if (robot_on_ball_line)
    {
        Vector ball_to_robot = robot->position() - ball.position();
        double dist_to_ball  = distance(robot->position(), ball.position());
        double dist_in_front_of_ball_to_intercept =
            std::max<double>(dist_to_ball - 1.0, 0.0);
        Point point_to_meet_ball =
            ball.position() + ball_to_robot.normalize(dist_in_front_of_ball_to_intercept);
        yield(std::make_unique<MoveIntent>(
            robot->id(), point_to_meet_ball,
            (ball.position() - robot->position()).orientation(), 0, 0, DribblerEnable::ON,
            MoveType::NORMAL, AutokickType::NONE, BallCollisionType::ALLOW));
    }
    else
    {
        yield(std::make_unique<MoveIntent>(
            robot->id(), closest_point_on_ball_trajectory,
            (ball.position() - robot->position()).orientation(), 0, 0, DribblerEnable::ON,
            MoveType::NORMAL, AutokickType::NONE, BallCollisionType::ALLOW));
    }
}
