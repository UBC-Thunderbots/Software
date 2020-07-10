#include "software/ai/hl/stp/action/intercept_ball_action.h"

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/evaluation/robot.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/intent/stop_intent.h"
#include "software/geom/util.h"
#include "software/logger/logger.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/util/acute_angle.h"
#include "software/new_geom/util/closest_point.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/contains.h"
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
    const bool ball_in_field = field.pointInFieldLines(ball.position());
    if (ball.velocity().length() == 0 && ball_in_field)
    {
        return std::nullopt;
    }
    Ray ball_ray(ball.position(), ball.velocity());
    if (ball_in_field)
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

void InterceptBallAction::interceptSlowBall(IntentCoroutine::push_type &yield) {
    bool intercept_done = false;
    do {
        auto face_ball_orientation = (ball.position() - robot->position()).orientation();
        yield(std::make_unique<MoveIntent>(
                robot->id(), ball.position(),
                face_ball_orientation, 0, 0,
                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
                BallCollisionType::ALLOW));

        bool ball_very_close_to_robot_dribbler = distance(ball.position(), robot->position() + Vector::createFromAngle(robot->orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) < 0.2 + BALL_MAX_RADIUS_METERS;
        Vector dribbler_tangential_velocity = Vector::createFromAngle(robot->orientation()).perpendicular().normalize((DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS) * robot->angularVelocity().toRadians());
        Vector expected_ball_velocity = robot->velocity() + dribbler_tangential_velocity;
        bool velocity_magnitudes_similar = std::fabs(expected_ball_velocity.length() - ball.velocity().length()) < 0.05;
        bool velocity_angles_similar = expected_ball_velocity.orientation().minDiff(ball.velocity().orientation()) < Angle::fromDegrees(10);
        bool ball_velocity_similar_to_robot = velocity_angles_similar && velocity_magnitudes_similar;
        intercept_done = ball_very_close_to_robot_dribbler && ball_velocity_similar_to_robot;
        if(ball.velocity().length() > BALL_MOVING_SLOW_SPEED_THRESHOLD * 2) {
            restart();
        }
    }while(!intercept_done);

    LOG(INFO) << "SLOW BALL: INITIAL INTERCEPT DONE\n\n\n";

    do {
//        auto face_ball_orientation = (ball.position() - robot->position()).orientation();

//        yield(std::make_unique<MoveIntent>(
//                robot->id(), robot->position(),
//                face_ball_orientation, 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
        yield(std::make_unique<StopIntent>(robot->id(), false, 0));
    }while(robot->velocity().length() > 0.01);

    LOG(INFO) << "SLOW BALL: WSTOPPED WITH BALL \n\n\n";
}

void InterceptBallAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    if(ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD) {
        interceptSlowBall(yield);
    }else {
        Point intercept_position = ball.position();
        while (contains(field.fieldLines(), intercept_position)) {
            Duration ball_time_to_position = Duration::fromSeconds(
                    distance(intercept_position, ball.position()) / (ball.velocity().length() + 1e-6));
            Duration robot_time_to_pos = getTimeToPositionForRobot(
                    robot->position(), intercept_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                    ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

            if (robot_time_to_pos < ball_time_to_position) {
                break;
            }
            intercept_position += ball.velocity().normalize(0.1);
        }

        LOG(INFO) << "FAST BALL: INTERCEPTING MOVING BALL AT " << intercept_position;

        bool robot_in_intercept_position = false;
        do {
            yield(std::make_unique<MoveIntent>(
                    robot->id(), intercept_position,
                    (-ball.velocity()).orientation(), 0, 0,
                    DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
                    BallCollisionType::AVOID));

            bool intercept_point_in_front_of_ball =
                    acuteAngle(ball.velocity(), intercept_position - ball.position()) < Angle::quarter();
            bool robot_at_intercept_point = distance(robot->position(), intercept_position) < 0.015;
            bool robot_stopped = robot->velocity().length() < 0.02;
            robot_in_intercept_position = intercept_point_in_front_of_ball && robot_at_intercept_point && robot_stopped;

            if(ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD / 2.0) {
                restart();
            }
        } while (!robot_in_intercept_position);

        LOG(INFO) << "FAST BALL: ROBOT IN POSITION " << intercept_position;

        bool intercept_done = false;
        do {
            yield(std::make_unique<MoveIntent>(
                    robot->id(), intercept_position,
                    (-ball.velocity()).orientation(), 0, 0,
                    DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
                    BallCollisionType::ALLOW));

            if(!intercept_done && ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD / 2.0) {
                restart();
            }
            bool ball_very_close_to_robot_dribbler = distance(ball.position(), robot->position() + Vector::createFromAngle(robot->orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) < 0.2 + BALL_MAX_RADIUS_METERS;
//            Vector dribbler_tangential_velocity = Vector::createFromAngle(robot->orientation()).perpendicular().normalize((DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS) * robot->angularVelocity().toRadians());
//            Vector expected_ball_velocity = robot->velocity() + dribbler_tangential_velocity;
//            bool velocity_magnitudes_similar = std::fabs(expected_ball_velocity.length() - ball.velocity().length()) < 0.05;
//            bool velocity_angles_similar = expected_ball_velocity.orientation().minDiff(ball.velocity().orientation()) < Angle::fromDegrees(10);
//            bool ball_velocity_similar_to_robot = velocity_angles_similar && velocity_magnitudes_similar;
            intercept_done = ball_very_close_to_robot_dribbler;
        }while(!intercept_done);

        LOG(INFO) << "FAST BALL: BALL IN DRIBBLER " << intercept_position;
    }

    LOG(INFO) << "DONE INTERCEPTING BALL";

//    while(true) {
//        yield(std::make_unique<MoveIntent>(
//                robot->id(), robot->position(),
//                robot->orientation(), 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
//    }
//



//
//        Point intercept_position = ball.position();
//        while(contains(field.fieldLines(), intercept_position)) {
//            Duration ball_time_to_position = Duration::fromSeconds(
//                    distance(intercept_position, ball.position()) / (ball.velocity().length() + 1e-6));
//            Duration robot_time_to_pos = getTimeToPositionForRobot(
//                    robot->position(), intercept_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
//                    ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
//
//            if(robot_time_to_pos < ball_time_to_position) {
//                break;
//            }
//            intercept_position += ball.velocity().normalize(0.1);
//        }
//
//        LOG(INFO) << "INTERCEPTING MOVING BALL AT " << intercept_position;
//        yield(std::make_unique<MoveIntent>(
//                robot->id(), intercept_position,
//                (-ball.velocity()).orientation(), 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
//
//
//
//
//
//    }
//
//    bool intercept_done = false;
//    do {
//        auto face_ball_orientation = (ball.position() - robot->position()).orientation();
//        if (ball.velocity().length() < 0.3) {
//            yield(std::make_unique<MoveIntent>(
//                    robot->id(), ball.position(),
//                    face_ball_orientation, 0, 0,
//                    DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                    BallCollisionType::ALLOW));
//        }else {
//            Point intercept_position = ball.position();
//            while(contains(field.fieldLines(), intercept_position)) {
//                Duration ball_time_to_position = Duration::fromSeconds(
//                        distance(intercept_position, ball.position()) / (ball.velocity().length() + 1e-6));
//                Duration robot_time_to_pos = getTimeToPositionForRobot(
//                        robot->position(), intercept_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
//                        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
//
//                if(robot_time_to_pos < ball_time_to_position) {
//                    break;
//                }
//                intercept_position += ball.velocity().normalize(0.1);
//            }
//
//            LOG(INFO) << "INTERCEPTING MOVING BALL AT " << intercept_position;
//
//            bool robot_in_intercept_position = false;
//            do {
//                yield(std::make_unique<MoveIntent>(
//                        robot->id(), intercept_position,
//                        (-ball.velocity()).orientation(), 0, 0,
//                        DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                        BallCollisionType::ALLOW));
//
//                bool intercept_point_in_front_of_ball = acuteAngle(ball.velocity(), intercept_position - ball.position()) < Angle::quarter();
//                bool robot_at_intercept_point = distance(robot->position(), intercept_position) < 0.015;
//                bool robot_stopped = robot->velocity().length() < 0.02;
//                robot_in_intercept_position = intercept_point_in_front_of_ball && robot_at_intercept_point && robot_stopped;
//            }while(!robot_in_intercept_position);
//        }
//
//
//
//
////
////        bool ball_very_close_to_robot_dribbler = distance(ball.position(), robot->position() + Vector::createFromAngle(robot->orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) < 0.2 + BALL_MAX_RADIUS_METERS;
//        Vector dribbler_tangential_velocity = Vector::createFromAngle(robot->orientation()).perpendicular().normalize((DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS) * robot->angularVelocity().toRadians());
//        Vector expected_ball_velocity = robot->velocity() + dribbler_tangential_velocity;
//        bool velocity_magnitudes_similar = std::fabs(expected_ball_velocity.length() - ball.velocity().length()) < 0.05;
//        bool velocity_angles_similar = expected_ball_velocity.orientation().minDiff(ball.velocity().orientation()) < Angle::fromDegrees(10);
//        bool ball_velocity_similar_to_robot = velocity_angles_similar && velocity_magnitudes_similar;
//        intercept_done = ball_very_close_to_robot_dribbler && ball_velocity_similar_to_robot;
//    }while(!intercept_done);
//
//    LOG(INFO) << "INITIAL INTERCEPT DONE\n\n\n";
//
//    do {
//        auto face_ball_orientation = (ball.position() - robot->position()).orientation();
//
//        yield(std::make_unique<MoveIntent>(
//                robot->id(), robot->position(),
//                face_ball_orientation, 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
////        }
////        bool ball_very_close_to_robot_dribbler = distance(ball.position(), robot->position() + Vector::createFromAngle(robot->orientation()).normalize(DIST_TO_FRONT_OF_ROBOT_METERS)) < 0.2 + BALL_MAX_RADIUS_METERS;
////        Vector dribbler_tangential_velocity = Vector::createFromAngle(robot->orientation()).perpendicular().normalize((DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS) * robot->angularVelocity().toRadians());
////        Vector expected_ball_velocity = robot->velocity() + dribbler_tangential_velocity;
////        bool velocity_magnitudes_similar = std::fabs(expected_ball_velocity.length() - ball.velocity().length()) < 0.05;
////        bool velocity_angles_similar = expected_ball_velocity.orientation().minDiff(ball.velocity().orientation()) < Angle::fromDegrees(10);
////        bool ball_velocity_similar_to_robot = velocity_angles_similar && velocity_magnitudes_similar;
////        intercept_done = ball_very_close_to_robot_dribbler && ball_velocity_similar_to_robot;
//    }while(robot->velocity().length() > 0.02);
//
//    LOG(INFO) << "STOPPED WITH BALL \n\n\n";
//
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
//    do
//    {
//        Point closest_point = ball.position();
//        if (ball.velocity().length() != 0)
//        {
//            closest_point = closestPointOnLine(
//                robot->position(),
//                Line(ball.position(), ball.position() + ball.velocity()));
//        }
//        bool point_in_front_of_ball =
//            acuteAngle(ball.velocity(), closest_point - ball.position()) <
//            Angle::quarter();
//
//        // We add 1e-6 to avoid division by 0 without affecting the result significantly
//        Duration ball_time_to_position = Duration::fromSeconds(
//            distance(closest_point, ball.position()) / (ball.velocity().length() + 1e-6));
//        Duration robot_time_to_pos = getTimeToPositionForRobot(
//            robot->position(), closest_point, ROBOT_MAX_SPEED_METERS_PER_SECOND,
//            ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
//
//        std::optional<Point> intercept_pos = std::nullopt;
//        if (point_in_front_of_ball && (ball_time_to_position > robot_time_to_pos))
//        {
//            intercept_pos = closest_point;
//        }
//
//        auto point_ball_leaves_field = getPointBallLeavesField(field, ball);
//        if (intercept_pos)
//        {
//            while (point_in_front_of_ball)
//            {
//                moveToInterceptPosition(yield, closest_point);
//
//                closest_point = ball.position();
//                if (ball.velocity().length() != 0){
//                    closest_point = closestPointOnLine(
//                        robot->position(),
//                        Line(ball.position(), ball.position() + ball.velocity()));
//                }
//                point_in_front_of_ball =
//                    acuteAngle(ball.velocity(), closest_point - ball.position()) <
//                    Angle::quarter();
//            }
//        }
//        else if (point_ball_leaves_field)
//        {
//            LOG(DEBUG) << "ball leaving field" << std::endl;
//            yield(std::make_unique<MoveIntent>(
//                robot->id(), point_ball_leaves_field.value(),
//                (ball.position() - robot->position()).orientation(), 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
//        }
//        else
//        {
//            // This is a fallback case that ideally should never be reached. We will only
//            // enter this case if the robot is not in front of the ball, and the ball is
//            // not within the field.
//            LOG(DEBUG) << "Moving to ball backup case" << std::endl;
//            yield(std::make_unique<MoveIntent>(
//                robot->id(), ball.position(),
//                (ball.position() - robot->position()).orientation(), 0, 0,
//                DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//                BallCollisionType::ALLOW));
//        }
//    } while (!robotHasPossession(ball.getPreviousStates(), robot->getPreviousStates()));
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
        // Move to a point near the ball
        while (distance(ball.position(), getRobot()->position()) >
                   ROBOT_MAX_RADIUS_METERS * 1.2 ||
               (ball.position() - robot->position())
                       .orientation()
                       .minDiff(getRobot()->orientation())
                       .abs() > Angle::fromDegrees(3))
        {
            Point target_position =
                ball.position() + (robot->position() - ball.position()).normalize() *
                                      ROBOT_MAX_RADIUS_METERS * 1.1;
            yield(std::make_unique<MoveIntent>(
                robot->id(), target_position,
                (ball.position() - robot->position()).orientation(),
                FINAL_SPEED_AT_SLOW_BALL, 0, DribblerEnable::ON, MoveType::NORMAL,
                AutokickType::NONE, BallCollisionType::ALLOW));
        }

        // Slowly approach the ball with the dribbler on
        while(ball.velocity().length() < 0.05){
            yield(std::make_unique<MoveIntent>(
                robot->id(), ball.position(),
                (ball.position() - robot->position()).orientation(),
                ball.velocity().length(), 0, DribblerEnable::ON, MoveType::NORMAL,
                AutokickType::NONE, BallCollisionType::ALLOW));
        }


        // TODO: comment here
        if (ball.velocity().length() < 0.1){
            // Once ball speed increases, bring the robot to a stop with the dribbler on
            while(ball.velocity().length() > 0.01){
                yield(std::make_unique<MoveIntent>(
                    robot->id(), getRobot()->position(),
                    robot->orientation(),
                    0, 0, DribblerEnable::ON, MoveType::NORMAL,
                    AutokickType::NONE, BallCollisionType::ALLOW));
            }
        }

        // Run through the ball
//        yield(std::make_unique<MoveIntent>(
//            robot->id(), ball.position(),
//            (ball.position() - robot->position()).orientation(), FINAL_SPEED_AT_SLOW_BALL,
//            0, DribblerEnable::ON, MoveType::NORMAL, AutokickType::NONE,
//            BallCollisionType::ALLOW));
    }
    else if (robot_on_ball_line)
    {
        // Move to a point on the line such that the ball runs into the front of the robot
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
