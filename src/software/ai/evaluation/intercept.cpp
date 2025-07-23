#include "software/ai/evaluation/intercept.h"

#include "shared/constants.h"
#include "software/ai/evaluation/time_to_travel.h"
#include "software/geom/algorithms/contains.h"
#include "software/optimization/gradient_descent_optimizer.hpp"

std::optional<std::pair<Point, Duration>> findBestInterceptForBall(const Ball &ball,
                                                                   const Field &field,
                                                                   const Robot &robot)
{
    static const double gradient_approx_step_size = 0.000001;

    // We use this to take a smooth absolute value in our objective function
    static const double smooth_abs_eps = 1000 * gradient_approx_step_size;

    // This is the objective function that we want to minimize, finding the
    // shortest duration in the future at which we can feasibly intercept the
    // ball
    auto objective_function = [&](std::array<double, 1> x)
    {
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
        Duration time_to_ball_pos = robot.getTimeToPosition(new_ball_pos);

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

    Point best_ball_intercept_pos =
        ball.estimateFutureState(best_ball_travel_duration).position();

    // Check that we can get to the best position in time
    Duration time_to_ball_pos     = robot.getTimeToPosition(best_ball_intercept_pos);
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

/**
 * Attempts to find a reachable overshoot destination for intercepting the ball,
 * adjusting final speed in steps up to the robot's max speed.
 *
 * @param robot The robot attempting to intercept.
 * @param base_position The ideal intercept point without overshoot.
 * @param ball_intercept_time Time the ball will take to reach the base position.
 * @param step_speed Speed increment for each overshoot iteration.
 * @param restrict_to_defense_area Whether to restrict overshoot to the friendly defense area.
 * @return The best reachable intercept point (possibly overshot), or base_position if no improvement.
 */
Point findOvershootInterceptPosition(
    const Robot& robot,
    const Point intercept_position,
    const Field& field,
    Duration ball_intercept_time,
    double step_speed,
    bool restrict_to_defense_area)
{
    Point new_destination = intercept_position;
    Point base_position = intercept_position;
    Point best_position    = intercept_position;
    double final_speed     = step_speed;
    bool finished          = false;
    double max_speed       = robot.robotConstants().robot_max_speed_m_per_s;
    double max_acc         = robot.robotConstants().robot_max_acceleration_m_per_s_2;

    while (!finished)
    {
        Vector final_velocity =
            (new_destination - robot.position()).normalize(final_speed);

        double extra_length = (final_speed * final_speed) / (2.0 * max_acc);
        new_destination      = base_position + final_velocity.normalize(extra_length);

        bool in_bounds = contains(field.fieldBoundary(), new_destination);
        bool in_defense = field.pointInFriendlyDefenseArea(new_destination);

        if ((!restrict_to_defense_area ^ in_defense) && in_bounds)
        {
            if(restrict_to_defense_area){
                best_position = new_destination;
            }

            if (robot.getTimeToPosition(base_position, final_velocity) < ball_intercept_time)
            {
                best_position = new_destination;
                finished      = true;
            }
        }
        else
        {
            finished = true;
        }

        final_speed += step_speed;
        if (final_speed > max_speed)
        {
            finished = true;
        }
    }

    return best_position;
}
