/**
 * Implementation for STP intercept-related evaluation functions
 */
#include "intercept.h"

#include "ai/evaluation/pass.h"
#include "shared/constants.h"
#include "util/gradient_descent.h"

std::optional<std::pair<Point, Duration>> findBestInterceptForBall(Ball ball, Field field,
                                                                   Robot robot)
{
    // This is the function that we want to minimize, finding the shortest duration in
    // the future at which we can feasibly intercept the ball
    auto objective_function = [&](std::array<double, 1> x) {
        // We take the absolute value here because a negative time makes no sense
        double duration = std::abs(x.at(0));

        // If the ball timestamp is less then the robot timestamp, add the difference here
        // so that we're optimizing to a duration that is after the robot timestamp
        if (ball.lastUpdateTimestamp() < robot.lastUpdateTimestamp())
        {
            duration +=
                (robot.lastUpdateTimestamp() - ball.lastUpdateTimestamp()).getSeconds();
        }

        // Estimate the ball position
        Point new_ball_pos =
            ball.estimatePositionAtFutureTime(Duration::fromSeconds(duration));

        // Figure out how long it will take the robot to get to the new ball position
        Duration time_to_ball_pos = AI::Evaluation::getTimeToPositionForRobot(
            robot, new_ball_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
            ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        // Figure out when the robot will reach the new ball position relative to the
        // time that the ball will get there (ie. will we get there in time?)
        double ball_robot_time_diff = duration - time_to_ball_pos.getSeconds();

        // Figure out how soon we can get to the ball, but also accounting for the time
        // it will take the robot to make it to the destination. This ensures we're
        // minimizing so that we get to the ball as soon as possible
        return duration - ball_robot_time_diff;
    };

    // Figure out when/where to intercept the ball. We do this by optimizing over
    // the ball position as a function of it's travel time
    // We make the weight here an inverse of the ball speed, so that the gradient descent
    // takes smaller steps when the ball is moving faster
    // We add a small amount to the ball velocity here to prevent division by 0, and also
    // to prevent the weight from getting two small, which would make gradient descent
    // take really large time steps
    double descent_weight = 1 / (ball.velocity().len() + 0.01);
    Util::GradientDescentOptimizer<1> optimizer({descent_weight}, 0.001);
    Duration best_ball_travel_duration = Duration::fromSeconds(
        std::abs(optimizer.minimize(objective_function, {0}, 100).at(0)));

    // In the objective function above, if the robot timestamp > ball timestamp, we
    // add on the difference so we get a intercept time after the robot timestamp, so we
    // need to do the same here to get the duration we actually optimized on
    if (robot.lastUpdateTimestamp() > ball.lastUpdateTimestamp())
    {
        best_ball_travel_duration =
            best_ball_travel_duration +
            (robot.lastUpdateTimestamp() - ball.lastUpdateTimestamp());
    }

    Point best_ball_intercept_pos =
        ball.estimatePositionAtFutureTime(best_ball_travel_duration);

    // Check that we can get to the best position in time
    Duration time_to_ball_pos = AI::Evaluation::getTimeToPositionForRobot(
        robot, best_ball_intercept_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    // NOTE: We have an addtional check here for 0 ball velocity
    if (ball.velocity().len() != 0 && time_to_ball_pos > best_ball_travel_duration)
    {
        return std::nullopt;
    }

    // Check that the best intercept position is actually on the field
    if (!field.pointInFieldLines(best_ball_intercept_pos))
    {
        return std::nullopt;
    }

    return std::make_pair(best_ball_intercept_pos, time_to_ball_pos);
}
