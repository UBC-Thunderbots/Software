#include "software/ai/evaluation/intercept.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_on_ball_path_ball_3_m_per_s)
{
    // Test where the robot is just sitting on the path the ball is travelling along
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {3, 0}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_LE(2 / 3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_on_ball_path_ball_6_m_per_s)
{
    // This is the max speed the ball should ever be traveling at
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {6, 0}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(1.75, intercept_pos.x());
    EXPECT_GE(2.25, intercept_pos.x());
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_LE(2 / 3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_right_beside_ball_path)
{
    // Test where the robot is sitting just off to the side of the path the ball
    // is travelling along
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {1, 0}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 0.2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(2, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_chasing_ball)
{
    // Test where the ball starts ahead of the robot, but moving slowly so that
    // the robot can catch up to it. Both the robot and the ball start at one
    // end of the field moving towards the other end
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 3}, {0, -0.5}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({0, 4}, {0, -1}, Angle::zero(), AngularVelocity::zero(),
                                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect the intercept will occur somewhere in x:[1,2], y:[1,2], t:[0, 3]
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_GE(3, intercept_pos.y());
    EXPECT_LE(1, intercept_pos.y());
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_on_diagonal_trajectory)
{
    // Test where the robot is sitting just off to the side of the path the ball
    // is travelling along
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0.5, 0.5}, {0.5, 0.5}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({3, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_not_moving)
{
    // Test where the ball is not moving
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({-2, -1}, {0, 0}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(5, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_moving_very_slowly)
{
    // Test where the ball is moving very slowly
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({-2, -1}, {0.000001, 0.000001}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
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
    EXPECT_LE(0, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(5, robot_time_to_move_to_intercept.getSeconds());
}


TEST(InterceptEvaluationTest, findBestInterceptForBall_robot_timestamp_ahead_of_ball)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                                Timestamp::fromSeconds(5));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_timestamp_ahead_of_robot)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    TimestampedRobotState robot({2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                                Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_non_zero_robot_and_ball_timestamp)
{
    // Test where the robot has a timestamp that is significantly greater then that
    // of the ball
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    TimestampedRobotState robot({2, 2}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                                Timestamp::fromSeconds(7));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will take the robot about 2 seconds
    auto [intercept_pos, robot_time_to_move_to_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_DOUBLE_EQ(0, intercept_pos.x());
    EXPECT_LE(1, robot_time_to_move_to_intercept.getSeconds());
    EXPECT_GE(3, robot_time_to_move_to_intercept.getSeconds());
}

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_moving_too_fast_to_intercept)
{
    // Test where we cannot get to the ball before it leaves the field, but if we ignore
    // field boundaries we could eventually catch it
    Field field = Field::createSSLDivisionBField();
    TimestampedBallState ball({3, 3}, {1, 1}, Timestamp::fromSeconds(0));
    TimestampedRobotState robot({2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                                Timestamp::fromSeconds(0));

    // We don't expect to be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_FALSE(best_intercept);
}


TEST(InterceptBallAtFutureTimeTest,
     get_position_at_future_time_with_positive_ball_velocity)
{
    Ball ball = Ball(Point(), Vector(1, 2), Timestamp::fromSeconds(123));

    EXPECT_EQ(Point(0.15, 0.3),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(150)));
    EXPECT_EQ(Point(1, 2),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(2, 4),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(2000)));
}

TEST(InterceptBallAtFutureTimeTest,
     get_position_at_future_time_with_negative_ball_velocity)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), Timestamp::fromSeconds(123));

    EXPECT_EQ(Point(2.325, 6.982),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(150)));
    EXPECT_EQ(Point(-1.5, 6.88),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(1000)));
    EXPECT_EQ(Point(-6, 6.76),
              estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                               Duration::fromMilliseconds(2000)));
}

TEST(InterceptBallAtFutureTimeTest, get_position_at_past_time)
{
    Ball ball = Ball(Point(3, 7), Vector(1, 0.12), Timestamp::fromSeconds(123));

    ASSERT_THROW(estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                                  Duration::fromMilliseconds(-200)),
                 std::invalid_argument);
    ASSERT_THROW(estimateBallPositionAtFutureTime(ball.position(), ball.velocity(),
                                                  Duration::fromMilliseconds(-2000)),
                 std::invalid_argument);
}
