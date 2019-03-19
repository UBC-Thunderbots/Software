#include "shared/constants.h"
#include "util/gradient_descent.h"
#include "ai/evaluation/pass.h"
#include "pass.h"

std::optional <std::pair<Point, Duration>> findBestInterceptForBall(Ball ball, Field field, Robot robot) {
    // This is the function that we want to minimize, finding the shortest duration in
    // the future at which we can feasibly intercept the ball
    auto objective_function = [&](std::array<double, 1> x){
        double duration = x.at(0);

        // Estimate the ball position
        Point new_ball_pos = ball.estimatePositionAtFutureTime(Duration::fromSeconds(duration));

        // Figure out how long it will take the robot to get to the new ball position
        Duration time_to_ball_pos = AI::Evaluation::getTimeToPositionForRobot(
                robot, new_ball_pos,
                ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                );

        // TODO: handle timestamp differences between the ball and robot, should have unit tests for this!

        // Figure out when the robot will reach the new ball position relative to the
        // time that the ball will get there (ie. will we get there in time?)
        // TODO: NAME THIS PROPERLY
        double a = duration - time_to_ball_pos.getSeconds();

        // Figure out how soon we can get to the ball, but also accounting for the time
        // it will take the robot to make it to the destination
        return duration - a;
    };

    // Figure out when/where to intercept the ball. We do this by optimizing over
    // the ball position as a function of it's travel time
    Util::GradientDescentOptimizer<1> optimizer;
    Duration best_ball_travel_duration = Duration::fromSeconds(optimizer.minimize(objective_function, {0}, 10).at(0));
    Point best_ball_intercept_pos = ball.estimatePositionAtFutureTime(best_ball_travel_duration);

    // Check that we can get to the best position in time
    // TODO: handle timestamp differences between ball and robot here
    Duration time_to_ball_pos = AI::Evaluation::getTimeToPositionForRobot(
            robot, best_ball_intercept_pos,
            ROBOT_MAX_SPEED_METERS_PER_SECOND,
            ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
    );
    if (time_to_ball_pos > best_ball_travel_duration){
        return std::nullopt;
    }

    // Check that the best intercept position is actually on the field
    if (!field.pointInPlayableArea(best_ball_intercept_pos)){
        return std::nullopt;
    }

    return std::make_pair(best_ball_intercept_pos, time_to_ball_pos);
}
