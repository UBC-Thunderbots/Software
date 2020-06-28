#include "software/ai/evaluation/intercept.h"

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/optimization/gradient_descent_optimizer.h"

std::optional<std::pair<Point, Duration>> findBestInterceptForBall(
    TimestampedBallState ball, Field field, TimestampedRobotState robot)
{
    static const double gradient_approx_step_size = 0.000001;

    // We use this to take a smooth absolute value in our objective function
    static const double smooth_abs_eps = 1000 * gradient_approx_step_size;

    // This is the objective function that we want to minimize, finding the
    // shortest duration in the future at which we can feasibly intercept the
    // ball
    auto objective_function = [&](std::array<double, 1> x) {
        // We take the absolute value here because a negative time makes no sense
        double duration = std::abs(x.at(0));

        // If the ball timestamp is less then the robot timestamp, add the difference
        // here so that we're optimizing to a duration that is after the robot
        // timestamp
        if (ball.timestamp() < robot.timestamp())
        {
            duration += (robot.timestamp() - ball.timestamp()).getSeconds();
        }

        // Estimate the ball position
        Point new_ball_pos = estimateBallPositionAtFutureTime(
            ball.state().position(), ball.state().velocity(),
            Duration::fromSeconds(duration));

        // Figure out how long it will take the robot to get to the new ball position
        Duration time_to_ball_pos = getTimeToPositionForRobot(
            robot.state().position(), new_ball_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
            ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        // Figure out when the robot will reach the new ball position relative to the
        // time that the ball will get there (ie. will we get there in time?)
        double ball_robot_time_diff = duration - time_to_ball_pos.getSeconds();

        // We want to get to the ball at the earliest opportunity possible, so
        // aim for a time diff of zero. We use a smooth approximation of
        // the maximum here
        return std::sqrt(std::pow(ball_robot_time_diff, 2) + smooth_abs_eps);
    };

    // Figure out when/where to intercept the ball. We do this by optimizing over
    // the ball position as a function of it's travel time
    // We make the weight here an inverse of the ball speed, so that the gradient
    // descent takes smaller steps when the ball is moving faster
    double descent_weight = 1 / (std::exp(ball.state().velocity().length() * 0.5));
    GradientDescentOptimizer<1> optimizer({descent_weight}, gradient_approx_step_size);
    Duration best_ball_travel_duration = Duration::fromSeconds(
        std::abs(optimizer.minimize(objective_function, {0}, 50).at(0)));

    // In the objective function above, if the robot timestamp > ball timestamp, we
    // add on the difference so we get a intercept time after the robot timestamp, so
    // we need to do the same here to get the duration we actually optimized on
    if (robot.timestamp() > ball.timestamp())
    {
        best_ball_travel_duration =
            best_ball_travel_duration + (robot.timestamp() - ball.timestamp());
    }

    Point best_ball_intercept_pos = estimateBallPositionAtFutureTime(
        ball.state().position(), ball.state().velocity(), best_ball_travel_duration);

    // Check that we can get to the best position in time
    Duration time_to_ball_pos =
        getTimeToPositionForRobot(robot.state().position(), best_ball_intercept_pos,
                                  ROBOT_MAX_SPEED_METERS_PER_SECOND,
                                  ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Duration ball_robot_time_diff = time_to_ball_pos - best_ball_travel_duration;
    // NOTE: if ball velocity is 0 then ball travel duration is infinite, so this
    // check isn't relevant in that case
    if (ball.state().velocity().length() != 0 &&
        std::abs(ball_robot_time_diff.getSeconds()) > descent_weight)
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

Point estimateBallPositionAtFutureTime(Point position, Vector velocity,
                                       const Duration &duration_in_future)
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Position estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/47
    double seconds_in_future = duration_in_future.getSeconds();
    return position + (velocity.normalize(seconds_in_future * velocity.length()));
}
