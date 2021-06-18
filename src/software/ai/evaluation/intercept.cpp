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
    if(false){
        return findInterceptionPoint(ball, field, robot);
    }
    static const double gradient_approx_step_size = 0.000001;

    // We use this to take a smooth absolute value in our objective function
    static const double smooth_abs_eps = 1000 * gradient_approx_step_size;

    // the speed at which we chase after the ball instead of intercepting
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;

    // the distance at which we chase after the ball instead of intercepting
    static constexpr double BALL_MIN_DISTANCE_TO_CHASE   = 0.3;

    Duration best_ball_travel_duration;
    Point best_ball_intercept_pos;

    //at small enough speed or distance its more efficient to just move to the ball
    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD || distance(ball.position(), robot.position()) < BALL_MIN_DISTANCE_TO_CHASE)
    {
        Point adjusted_ball_position = ball.position();

        if (robot.timestamp() > ball.timestamp())
        {
            auto time_difference = robot.timestamp().toSeconds() - ball.timestamp().toSeconds();
            adjusted_ball_position = ball.estimateFutureState(Duration::fromSeconds(time_difference)).position();
        }
        auto face_ball_vector = (ball.position() - robot.position());

        auto point_in_front_of_ball =
                adjusted_ball_position -
                Vector::createFromAngle(face_ball_vector.orientation())
                        .normalize(DIST_TO_FRONT_OF_ROBOT_METERS);

        best_ball_intercept_pos = point_in_front_of_ball;
        best_ball_travel_duration = getTimeToPositionForRobot(
                robot.position(), best_ball_intercept_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

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
                robot.position(), new_ball_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);


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

    auto future_state = ball.estimateFutureState(best_ball_travel_duration);
    best_ball_intercept_pos =
        future_state.position();

//     Check that we can get to the best position in time
    Duration time_to_ball_pos = getTimeToPositionForRobot(
        robot.position(), best_ball_intercept_pos, ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Duration ball_robot_time_diff = time_to_ball_pos - best_ball_travel_duration;

    // Check that the best intercept position is actually on the field
    if (!contains(field.fieldLines(), best_ball_intercept_pos))
    {
        return std::nullopt;
    }

    return std::make_pair(best_ball_intercept_pos, time_to_ball_pos);
}

//old implementation, will remove before pr
std::optional<std::pair<Point, Duration>> findInterceptionPoint(const Ball &ball, const Field &field, const Robot &robot)
{
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD   = 0.3;
    static constexpr double INTERCEPT_POSITION_SEARCH_INTERVAL = 0.1;


    if (ball.velocity().length() < BALL_MOVING_SLOW_SPEED_THRESHOLD)
    {
        auto face_ball_vector = (ball.position() - robot.position());

        auto point_in_front_of_ball =
                ball.position() -
                Vector::createFromAngle(face_ball_vector.orientation())
                        .normalize(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);
        Duration time_to_ball_pos = getTimeToPositionForRobot(
                robot.position(), point_in_front_of_ball, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        std::cout<<"early"<<std::endl;
        return std::make_pair(point_in_front_of_ball, time_to_ball_pos);
    }

    Point intercept_position = ball.position();
    Duration robot_time_to_pos;
    Point previous_intercept;
    while (contains(field.fieldLines(), intercept_position))
    {
        // at constant acceleration, final_speed^2 = initial_speed^2 + (acceleration *
        // displacement * 2)
        double final_ball_speed_at_position =
                std::sqrt(std::pow(ball.velocity().length(), 2) +
                          (2 * ball.acceleration().length() *
                           distance(intercept_position, ball.position())));

        // at constant acceleration, t = final_speed - initial_speed / acceleration
        Duration ball_time_to_position = Duration::fromSeconds(
                (final_ball_speed_at_position - ball.velocity().length()) /
                ball.acceleration().length());

//        Duration ball_time_to_position = Duration::fromSeconds(
//                (intercept_position - ball.position()).length() / ball.velocity().length());
        robot_time_to_pos = getTimeToPositionForRobot(
                robot.position(), intercept_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        if (robot_time_to_pos < ball_time_to_position)
        {
            break;
        }
        previous_intercept  = intercept_position;
        intercept_position +=
                ball.velocity().normalize(INTERCEPT_POSITION_SEARCH_INTERVAL);
    }

    // Check that the best intercept position is actually on the field
    if (!contains(field.fieldLines(), intercept_position))
    {
        std::cout<<"returned null"<<std::endl;
        std::make_pair(previous_intercept, robot_time_to_pos);
    }

    return std::make_pair(intercept_position, robot_time_to_pos);
}