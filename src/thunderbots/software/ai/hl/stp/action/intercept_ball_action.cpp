#include "ai/hl/stp/action/intercept_ball_action.h"
#include "ai/evaluation/pass.h"
#include "ai/hl/stp/evaluation/intercept.h"
#include "ai/hl/stp/evaluation/robot.h"
#include "util/logger/init.h"
#include "ai/evaluation/pass.h"
#include "ai/intent/move_intent.h"
#include "geom/ray.h"
#include "geom/util.h"
#include "shared/constants.h"

InterceptBallAction::InterceptBallAction(const Field& field, const Ball& ball, bool loop_forever)
    : Action(), field(field), ball(ball),
      loop_forever(loop_forever)
{
}

std::unique_ptr<Intent> InterceptBallAction::updateStateAndGetNextIntent(
    const Robot& robot, const Field& field, const Ball& ball)
{
    // Update the parameters stored by this Action
    this->field = field;
    this->ball = ball;
    this->robot = robot;

    return getNextIntent();
}

std::optional<Point> InterceptBallAction::getPointBallLeavesField(const Field &field, const Ball &ball) {
    Ray ball_ray(ball.position(), ball.velocity());
    if(field.pointInFieldLines(ball.position())) {
        return rayRectangleIntersection(ball_ray, field.fieldLines()).first;
    }else {
        return std::nullopt;
    }
}

void InterceptBallAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    do
    {
        Point closest_point = closestPointOnLine(robot->position(), ball.position(), ball.position() + ball.velocity());
        bool point_in_front_of_ball = pointInFrontVector(ball.position(), ball.velocity(), closest_point);

        Duration ball_time_to_position = Duration::fromSeconds(dist(closest_point, ball.position()) / (ball.velocity().len() + 1e-6));
        Duration robot_time_to_pos = AI::Evaluation::getTimeToPositionForRobot(*robot, closest_point, ROBOT_MAX_SPEED_METERS_PER_SECOND, ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        std::optional<Point> intercept_pos = std::nullopt;
        if(point_in_front_of_ball && (ball_time_to_position > robot_time_to_pos)) {
            intercept_pos = closest_point;
        }

        auto blf = getPointBallLeavesField(field, ball);
        if(intercept_pos) {
            while(point_in_front_of_ball) {
                bool robot_on_ball_line = pointInFrontVector(ball.position(), ball.velocity(), robot->position()) && dist(robot->position(), Line(ball.position(), ball.position() + ball.velocity())) < 0.02;
                LOG(INFO) << dist(robot->position(), Line(ball.position(), ball.position() + ball.velocity())) << std::endl;

                if(robot_on_ball_line) {
            Vector ball_to_robot = robot->position() - ball.position();
            double d = dist(robot->position(), ball.position());
            double d2 = std::min<double>(d-1.0, 0.0);
            Point point_to_meet_ball = ball.position() + ball_to_robot.norm(d2);
            yield(std::make_unique<MoveIntent>(robot->id(), point_to_meet_ball, (ball.position() - robot->position()).orientation(),
                                               0, 0, true, NONE));
                }else {
                                    yield(std::make_unique<MoveIntent>(robot->id(), closest_point, (ball.position() - robot->position()).orientation(),
                                                   0, 0, true, NONE));
                }



                closest_point = closestPointOnLine(robot->position(), ball.position(), ball.position() + ball.velocity());
                point_in_front_of_ball = pointInFrontVector(ball.position(), ball.velocity(), closest_point);
            }
        }
        else if(blf){
            yield(std::make_unique<MoveIntent>(robot->id(), blf.value(), (ball.position() - robot->position()).orientation(),
                                           0, 0, true, NONE));
        }else {
            yield(std::make_unique<MoveIntent>(robot->id(), ball.position(), (ball.position() - robot->position()).orientation(),
                                               0, 0, true, NONE));
        }
    } while (!Evaluation::robotHasPossession(ball, *robot) || loop_forever);
}
