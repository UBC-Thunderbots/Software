#include "software/ai/evaluation/intercept.h"

#include <gtest/gtest.h>

#include <random>

#include "software/geom/algorithms/contains.h"
#include "software/optimization/gradient_descent_optimizer.hpp"
#include "software/test_util/test_util.h"

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_on_ball_path_ball_3_m_per_s)
{
    // Test where the robot is just sitting on the path the ball is travelling along
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {3, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // The ball is travelling fast enough that we expect the best intercept to be
    // somewhere between x=1.75 and x=2.25 (with some tolerance allowed because of how we
    // do the optimisation here), with y = 0, and will occur sometime between 0 and
    // 2/3 seconds in the future (the time for the ball to reach the robots initial
    // position).
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(1.5, intercept_pos.x());
    EXPECT_GE(2.25, intercept_pos.x());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_LE(2 / 3, robot_time_to_move_to_intercept.toSeconds());
}

// TODO (#2540): Figure out why this is failing
TEST(InterceptEvaluationTest,
     DISABLED_findBestInterceptForBall_robot_on_ball_path_ball_6_m_per_s)
{
    // This is the max speed the ball should ever be traveling at
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {6, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0), std::set<RobotCapability>(),
                robot_constants::createRobotConstants());

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // The ball is travelling fast enough that we expect the best intercept to be
    // somewhere between x=1.75 and x=2.25 (with some tolerance allowed because of how we
    // do the optimisation here), with y = 0, and will occur sometime between 0 and
    // 2/3 seconds in the future (the time for the ball to reach the robots initial
    // position).
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(1.75, intercept_pos.x());
    EXPECT_GE(2.25, intercept_pos.x());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_LE(2 / 3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_right_beside_ball_path)
{
    // Test where the robot is sitting just off to the side of the path the ball
    // is travelling along
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {1, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2.25,
    // (with some extra tolerance because of how we do the optimisation)
    // with y = 0, and will occur sometime between 0 and 2 seconds in the future
    // (the time for the ball to reach the robot)
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(0, intercept_pos.x());
    EXPECT_GE(2.25, intercept_pos.x());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(2, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_chasing_ball)
{
    // Test where the ball starts ahead of the robot, but moving slowly so that
    // the robot can catch up to it. Both the robot and the ball start at one
    // end of the field moving towards the other end
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 3}, {0, -0.5}, Timestamp::fromSeconds(0));
    Robot robot(0, {0, 4}, {0, -1}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect the intercept will occur somewhere in x:[1,2], y:[1,2], t:[0, 3]
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_GE(3, intercept_pos.y());
    EXPECT_LE(1, intercept_pos.y());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_on_diagonal_trajectory)
{
    // Test where the robot is sitting just off to the side of the path the ball
    // is travelling along
    Field field = Field::createSSLDivisionBField();
    Ball ball({0.5, 0.5}, {0.5, 0.5}, Timestamp::fromSeconds(0));
    Robot robot(0, {3, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect the intercept will occur somewhere in x:[1,2], y:[1,2], t:[0, 3]
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_LE(1, intercept_pos.x());
    EXPECT_GE(2, intercept_pos.x());
    EXPECT_LE(1, intercept_pos.y());
    EXPECT_GE(2, intercept_pos.y());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_not_moving)
{
    // Test where the ball is not moving
    Field field = Field::createSSLDivisionBField();
    Ball ball({-2, -1}, {0, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // Since the ball is not moving, the only way to intercept it is to move to where
    // it is (at the origin)
    // The expected time is the time it will take the robot to move to the destination.
    // This is dependent on robot constants, but should be in [0,5] seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(-2, intercept_pos.x());
    EXPECT_DOUBLE_EQ(-1, intercept_pos.y());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(5, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_moving_very_slowly)
{
    // Test where the ball is moving very slowly
    Field field = Field::createSSLDivisionBField();
    Ball ball({-2, -1}, {0.000001, 0.000001}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // Since the ball is not moving, the only way to intercept it is to move to where
    // it is (at the origin)
    // The expected time is the time it will take the robot to move to the destination.
    // This is dependent on robot constants, but should be in [0,5] seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_NEAR(-2, intercept_pos.x(), 0.02);
    EXPECT_NEAR(-1, intercept_pos.y(), 0.02);
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(5, robot_time_to_move_to_intercept.toSeconds());
}


TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_timestamp_ahead_of_ball)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(5));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_timestamp_ahead_of_robot)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Robot robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_non_zero_robot_and_ball_timestamp)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Robot robot(0, {2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(7));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.toSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_moving_too_fast_to_intercept)
{
    // Test where we cannot get to the ball before it leaves the field, but if we ignore
    // field boundaries we could eventually catch it
    Field field = Field::createSSLDivisionBField();
    Ball ball({3, 3}, {1, 1}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    // We don't expect to be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_FALSE(best_intercept);
}

namespace
{
std::optional<std::pair<Point, Duration>> referenceIntercept(const Ball& ball,
                                                             const Field& field,
                                                             const Robot& robot)
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

}  // namespace

TEST(InterceptEvaluationTest, optimized_intercepts_match_numerical_reference)
{
    const auto field = Field::createSSLDivisionBField();
    std::mt19937 random(8041);
    std::uniform_real_distribution<double> position(-5, 5), velocity(-3, 3);
    for (int sample = 0; sample < 120; ++sample)
    {
        Vector ball_velocity =
            sample % 3 == 0 ? Vector() : Vector(velocity(random), velocity(random));
        Vector acceleration = sample % 5 == 0 ? Vector(0.3, -0.2) : Vector();
        Ball ball(Point(position(random), position(random)), ball_velocity,
                  Timestamp::fromSeconds(sample % 2), acceleration);
        Robot robot(0, Point(position(random), position(random)),
                    Vector(velocity(random), velocity(random)), Angle::zero(),
                    AngularVelocity::zero(), Timestamp::fromSeconds(sample % 2));
        auto expected = referenceIntercept(ball, field, robot);
        // The legacy solver accepted unreachable accelerating balls from rest.
        // Independently infer flight time from displacement to reject those results.
        if (expected && ball_velocity.x() == 0 && ball_velocity.y() == 0 &&
            acceleration.lengthSquared() > 0)
        {
            const double flight_time =
                std::sqrt(2 * (expected->first - ball.position()).dot(acceleration) /
                          acceleration.lengthSquared());
            if (std::abs(flight_time - expected->second.toSeconds()) > 1)
                expected = std::nullopt;
        }
        const auto actual = findBestInterceptForBall(ball, field, robot);
        ASSERT_EQ(actual.has_value(), expected.has_value()) << sample;
        if (actual)
        {
            EXPECT_DOUBLE_EQ(actual->first.x(), expected->first.x()) << sample;
            EXPECT_DOUBLE_EQ(actual->first.y(), expected->first.y()) << sample;
            EXPECT_DOUBLE_EQ(actual->second.toSeconds(), expected->second.toSeconds())
                << sample;
        }
    }
    for (const auto& point : {Point(0, 0), field.friendlyGoalCenter(),
                              Point(field.fieldLines().xMin() - 1e-9, 0)})
    {
        Ball ball(point, Vector(), Timestamp());
        Robot robot(0, point, Vector(), Angle::zero(), AngularVelocity::zero(),
                    Timestamp());
        EXPECT_EQ(findBestInterceptForBall(ball, field, robot),
                  referenceIntercept(ball, field, robot));
    }
}

TEST(InterceptEvaluationTest, stationary_ball_unreachable_by_zero_speed_robot)
{
    const auto field = Field::createSSLDivisionBField();
    auto constants   = robot_constants::createRobotConstants();
    constants.robot_trajectory_max_speed_m_per_s = 0;
    Robot robot(0, Point(2, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp(), false, {}, constants);
    Ball ball{Point(), Vector(), Timestamp()};
    EXPECT_FALSE(findBestInterceptForBall(ball, field, robot));
    Ball covered_ball(robot.position(), Vector(), Timestamp());
    const auto covered = findBestInterceptForBall(covered_ball, field, robot);
    ASSERT_TRUE(covered);
    EXPECT_DOUBLE_EQ(covered->second.toSeconds(), 0);
    EXPECT_FALSE(
        findBestInterceptForBall(Ball(Point(), Vector(2, 0), Timestamp()), field, robot));
}

TEST(InterceptEvaluationTest, tiny_motion_and_acceleration_do_not_use_stationary_shortcut)
{
    const auto field = Field::createSSLDivisionBField();
    Robot robot(0, Point(2, 1), Vector(0.2, -0.1), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(3));
    for (double magnitude : {-1e-9, 1e-9, -1e-4, 1e-4})
        for (bool accelerating : {false, true})
        {
            Ball ball(Point(0.2, -0.3), accelerating ? Vector() : Vector(magnitude, 0),
                      Timestamp::fromSeconds(3),
                      accelerating ? Vector(magnitude, 0) : Vector());
            const auto actual   = findBestInterceptForBall(ball, field, robot);
            const auto expected = referenceIntercept(ball, field, robot);
            ASSERT_TRUE(expected);
            ASSERT_TRUE(actual);
            EXPECT_DOUBLE_EQ(actual->first.x(), expected->first.x());
            EXPECT_DOUBLE_EQ(actual->first.y(), expected->first.y());
            EXPECT_DOUBLE_EQ(actual->second.toSeconds(), expected->second.toSeconds());
            EXPECT_NE(actual->first.x(), ball.position().x());
        }
}

TEST(InterceptEvaluationTest, stationary_ball_field_edges_and_timestamp_offsets)
{
    const auto field   = Field::createSSLDivisionBField();
    const auto& bounds = field.fieldLines();
    for (const auto& edge : {Point(bounds.xMin(), 0), Point(bounds.xMax(), 0),
                             Point(0, bounds.yMin()), Point(0, bounds.yMax())})
        for (bool outside : {false, true})
            for (double robot_time : {0.0, 10.0})
            {
                Point point =
                    outside ? Point(edge.x() * (1 + 1e-9), edge.y() * (1 + 1e-9)) : edge;
                Ball ball(point, Vector(-0.0, 0.0), Timestamp::fromSeconds(5),
                          Vector(0.0, -0.0));
                Robot robot(0, Point(1, 1), Vector(0.4, 0), Angle::zero(),
                            AngularVelocity::zero(), Timestamp::fromSeconds(robot_time));
                const auto actual   = findBestInterceptForBall(ball, field, robot);
                const auto expected = referenceIntercept(ball, field, robot);
                ASSERT_EQ(actual.has_value(), !outside);
                ASSERT_EQ(actual.has_value(), expected.has_value());
                if (actual)
                {
                    EXPECT_DOUBLE_EQ(actual->first.x(), expected->first.x());
                    EXPECT_DOUBLE_EQ(actual->first.y(), expected->first.y());
                    EXPECT_DOUBLE_EQ(actual->second.toSeconds(),
                                     expected->second.toSeconds());
                }
            }
}

TEST(InterceptEvaluationTest, stale_ball_observations_match_advancing_to_robot_timestamp)
{
    const auto field = Field::createSSLDivisionBField();
    Robot robot(0, Point(2, 1), Vector(0.2, -0.1), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(10));
    for (const auto& velocity : {Vector(1e-9, 0), Vector(0.02, 0)})
        for (const auto& acceleration : {Vector(), Vector(0.02, 0)})
        {
            Ball ball(Point(0.2, -0.3), velocity, Timestamp::fromSeconds(3),
                      acceleration);
            Ball advanced(ball.estimateFutureState(Duration::fromSeconds(7)),
                          robot.timestamp(), acceleration);
            const auto expected = referenceIntercept(advanced, field, robot);
            const auto actual   = findBestInterceptForBall(ball, field, robot);
            ASSERT_TRUE(expected);
            ASSERT_TRUE(actual);
            EXPECT_DOUBLE_EQ(actual->first.x(), expected->first.x());
            EXPECT_DOUBLE_EQ(actual->first.y(), expected->first.y());
            EXPECT_DOUBLE_EQ(actual->second.toSeconds(), expected->second.toSeconds());
        }
}

TEST(InterceptEvaluationTest, accelerating_from_rest_does_not_bypass_reachability)
{
    const auto field = Field::createSSLDivisionBField();
    std::mt19937 random(7193);
    std::uniform_real_distribution<double> position(-4, 4), acceleration(0.01, 10);
    unsigned int accepted = 0;
    for (int sample = 0; sample < 2000; ++sample)
    {
        const Point origin(position(random), 0);
        const double ax = acceleration(random);
        Ball ball(origin, Vector(), Timestamp(), Vector(ax, 0));
        Robot robot(0, Point(position(random), position(random)), Vector(), Angle::zero(),
                    AngularVelocity::zero(), Timestamp());
        auto result = findBestInterceptForBall(ball, field, robot);
        if (!result)
            continue;
        ++accepted;
        const double elapsed  = std::sqrt(2 * (result->first.x() - origin.x()) / ax);
        const double residual = std::abs(elapsed - result->second.toSeconds());
        if (residual > 1)
        {
            ADD_FAILURE() << "origin=" << origin << " robot=" << robot.position()
                          << " ax=" << ax << " target=" << result->first
                          << " elapsed=" << elapsed
                          << " travel=" << result->second.toSeconds();
            return;
        }
    }
    EXPECT_GT(accepted, 0);
}

TEST(InterceptEvaluationTest, rejects_accelerating_ball_that_arrives_before_robot)
{
    const auto field = Field::createSSLDivisionBField();
    Ball ball(Point(2.7474612912174319, 0), Vector(), Timestamp(),
              Vector(8.2467323948860756, 0));
    Robot robot(0, Point(2.9691508751050044, 1.8431293650314409), Vector(), Angle::zero(),
                AngularVelocity::zero(), Timestamp());
    EXPECT_FALSE(findBestInterceptForBall(ball, field, robot));
}
