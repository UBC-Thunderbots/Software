#include "software/ai/evaluation/intercept.h"

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/optimization/gradient_descent_optimizer.h"

std::optional<std::pair<Point, Duration>> findBestInterceptForBall(const Ball &ball,
                                                                   const Field &field,
                                                                   const Robot &robot)
{
    static const double gradient_approx_step_size = 0.000001;

    // We use this to take a smooth absolute value in our objective function
    static const double smooth_abs_eps = 1000 * gradient_approx_step_size;

    // the speed at which we chase after the ball instead of intercepting
    // we choose this number because its large enough to cover cases where
    // vision is unreliable and the ball is actually not moving (meaning we
    // need to go chase it)
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;

    // the distance at which we chase after the ball instead of intercepting
    // we use this because the object function behaves weirdly at small distances.
    static constexpr double BALL_MIN_DISTANCE_TO_CHASE = 0.8;

    Duration best_ball_travel_duration;
    Point best_ball_intercept_pos;

    // at small enough speed or distance its more efficient to just move to the ball
    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD ||
        distance(ball.position(), robot.position()) < BALL_MIN_DISTANCE_TO_CHASE)
    {
        Point adjusted_ball_position = ball.position();

        if (robot.timestamp() > ball.timestamp())
        {
            auto time_difference =
                robot.timestamp().toSeconds() - ball.timestamp().toSeconds();
            adjusted_ball_position =
                ball.estimateFutureState(Duration::fromSeconds(time_difference))
                    .position();
        }
        auto face_ball_vector = (ball.position() - robot.position());

        auto point_in_front_of_ball =
            adjusted_ball_position -
            Vector::createFromAngle(face_ball_vector.orientation())
                .normalize(DIST_TO_FRONT_OF_ROBOT_METERS);

        best_ball_intercept_pos   = point_in_front_of_ball;
        best_ball_travel_duration = getTimeToPositionForRobot(
            robot.position(), best_ball_intercept_pos, robot.robotConstants().robot_max_speed_m_per_s,
            robot.robotConstants().robot_max_acceleration_m_per_s_2);

        return std::make_pair(best_ball_intercept_pos, best_ball_travel_duration);
    }

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
            duration += (robot.timestamp() - ball.timestamp()).toSeconds();
        }

        // Estimate the ball position
        Point new_ball_pos =
            ball.estimateFutureState(Duration::fromSeconds(duration)).position();


        // Figure out how long it will take the robot to get to the new ball position
        Duration time_to_ball_pos = getTimeToPositionForRobot(
            robot.position(), new_ball_pos,
            robot.robotConstants().robot_max_speed_m_per_s,
            robot.robotConstants().robot_max_acceleration_m_per_s_2);


        // Figure out when the robot will reach the new ball position relative to the
        // time that the ball will get there (ie. will we get there in time?)
        double ball_robot_time_diff = duration - time_to_ball_pos.toSeconds();

        // We want to get to the ball at the earliest opportunity possible, so
        // aim for a time diff of zero. We use a smooth approximation of
        // the maximum here
        return std::sqrt(std::pow(ball_robot_time_diff, 2) + smooth_abs_eps);
    };

    // Figure out when/where to intercept the ball. We do this by optimizing over
    // the ball position as a function of it's travel time
    // We make the weight here an inverse of the ball speed, so that the gradient
    // descent takes smaller steps when the ball is moving faster
    double descent_weight = 1 / (std::exp(ball.currentState().velocity().length() * 0.5));
    GradientDescentOptimizer<1> optimizer({descent_weight}, gradient_approx_step_size);
    best_ball_travel_duration = Duration::fromSeconds(
        std::abs(optimizer.minimize(objective_function, {0}, 50).at(0)));

    // In the objective function above, if the robot timestamp > ball timestamp, we
    // add on the difference so we get a intercept time after the robot timestamp, so
    // we need to do the same here to get the duration we actually optimized on
    if (robot.timestamp() > ball.timestamp())
    {
        best_ball_travel_duration =
            best_ball_travel_duration + (robot.timestamp() - ball.timestamp());
    }

    auto future_state       = ball.estimateFutureState(best_ball_travel_duration);
    best_ball_intercept_pos = future_state.position();

    //     Check that we can get to the best position in time
    Duration time_to_ball_pos = getTimeToPositionForRobot(
        robot.position(), best_ball_intercept_pos,
        robot.robotConstants().robot_max_speed_m_per_s,
        robot.robotConstants().robot_max_acceleration_m_per_s_2);
    Duration ball_robot_time_diff = time_to_ball_pos - best_ball_travel_duration;
    // NOTE: if ball velocity is 0 then ball travel duration is infinite, so this
    // check isn't relevant in that case
    if (ball.currentState().velocity().length() != 0 &&
        std::abs(ball_robot_time_diff.toSeconds()) > descent_weight)
    {
        return std::nullopt;
    }

    // Check that the best intercept position is actually on the field
    if (!contains(field.fieldLines(), best_ball_intercept_pos))
    {
        return std::nullopt;
    }

    return std::make_pair(best_ball_intercept_pos, time_to_ball_pos);
}
