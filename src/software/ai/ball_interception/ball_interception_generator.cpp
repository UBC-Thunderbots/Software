#include "software/ai/ball_interception/ball_interception_generator.h"

#include "software/ai/evaluation/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

Point generateBallInterceptionPoint(const Field& field, const Ball& ball,
                                    const Robot& robot)
{
    static const double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;

    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        return interceptSlowBall(field, ball, robot);
    }
    else
    {
        return interceptFastBall(field, ball, robot);
    }
}

Point interceptSlowBall(const Field& field, const Ball& ball, const Robot& robot)
{
    //    static const double BLOCK_SLOW_BALL_BUFFER = 0.2;
    //    return ball.position() + ball.velocity().normalize(BLOCK_SLOW_BALL_BUFFER);
    return ball.position();
}

Point interceptFastBall(const Field& field, const Ball& ball, const Robot& robot)
{
    static const double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;
    // Find the first point where the robot can get to before the ball
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
