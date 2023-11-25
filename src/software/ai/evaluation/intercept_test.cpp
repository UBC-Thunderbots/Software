#include "software/ai/evaluation/intercept.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/geom/algorithms/distance.h"

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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
    
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(1.5, intercept_pos.x());
    EXPECT_GE(2.25, intercept_pos.x());
    EXPECT_LE(0, robot_time_to_move_to_intercept.toSeconds());
    EXPECT_LE(2 / 3, robot_time_to_move_to_intercept.toSeconds());
}

// TODO (#2540): Figure out why this is failing
TEST(InterceptEvaluationTest,
     findBestInterceptForBall_robot_on_ball_path_ball_6_m_per_s)
{
    // This is the max speed the ball should ever be traveling at
    Field field = Field::createSSLDivisionBField();
    Ball ball({0, 0}, {6, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2, 0}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0), std::set<RobotCapability>(),
                create2021RobotConstants());

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // The ball is travelling fast enough that we expect the best intercept to be
    // somewhere between x=1.75 and x=2.25 (with some tolerance allowed because of how we
    // do the optimisation here), with y = 0, and will occur sometime between 0 and
    // 2/3 seconds in the future (the time for the ball to reach the robots initial
    // position).
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;

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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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
    auto intercept_pos = best_intercept.value().point;
    auto robot_time_to_move_to_intercept = best_intercept.value().duration;
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

TEST(InterceptEvaluationTest, findBestInterceptForBall_ball_moving_too_fast_fallback)
{
    // Test where we cannot get to the ball before it leaves the field unless we intercept the ball while in motion
    Field field = Field::createSSLDivisionBField();
    Point edge_of_field = Point(4.5,0);
    Ball ball({3.75, 0}, {1.5, 0}, Timestamp::fromSeconds(0), {-0.15,0});
    double robot_max_speed = 4.0;
    double robot_max_acceleration = 6.5;
    double timetoedge_ball = distance(edge_of_field, ball.position())/ ball.velocity().length();

    // we find the position where the robot cant reach the ball unless it goes at max speed the entire time
    double robot_distance_away = timetoedge_ball * robot_max_speed;

    // decrease this distance so that the position is reachable in fallback interception only
    double reachable_distance_away = robot_distance_away - 1.3;

    Robot robot(0, {4.5, edge_of_field.y() - reachable_distance_away}, {0, 0}, Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));

    auto best_intercept_fallback = findBestInterceptForBall(ball, field, robot, true);
    auto best_intercept_no_fallback = findBestInterceptForBall(ball, field, robot, false);
    ASSERT_TRUE(best_intercept_fallback);
    ASSERT_FALSE(best_intercept_no_fallback);

    //expect the final speed to be somewhere around the final speed of the robot accelerating at max value to the edge
    double expected_final_speed = robot_max_acceleration * timetoedge_ball;
    ASSERT_TRUE(std::abs(expected_final_speed - best_intercept_fallback.value().final_speed) < 0.5);

}
