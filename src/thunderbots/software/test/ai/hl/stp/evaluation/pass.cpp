/**
 * This file contains the unit tests for evaluation functions
 * in pass.cpp
 */

#include <gtest/gtest.h>

#include "ai/hl/stp/evaluation/pass.h"
#include "test/test_util/test_util.h"

TEST(PassEvaluationTest, findBestInterceptForBall_robot_on_ball_path){
    // Test where the robot is just sitting on the path the ball is travelling along
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({0,0}, {1,0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2,0}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will occur sometime between 0 and 2 seconds in the future
    // (the time for the ball to reach the robot)
    auto [intercept_pos, duration_until_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(0, intercept_pos.x());
    EXPECT_GE(2, intercept_pos.x());

}

TEST(PassEvaluationTest, findBestInterceptForBall_robot_right_beside_ball_path){
    // Test where the robot is sitting just off to the side of the path the ball
    // is travelling along
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({0,0}, {1,0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2,0.2}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // We expect that the best intercept is going to be somewhere between x=0 and x=2,
    // with y = 0, and will occur sometime between 0 and 2 seconds in the future
    // (the time for the ball to reach the robot)
    auto [intercept_pos, duration_until_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(0, intercept_pos.y());
    EXPECT_LE(0, intercept_pos.x());
    EXPECT_GE(2, intercept_pos.x());

}

TEST(PassEvaluationTest, findBestInterceptForBall_ball_not_moving){
    // Test where the ball is not moving
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({7,13}, {0,0}, Timestamp::fromSeconds(0));
    Robot robot(0, {2,2}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We should be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_TRUE(best_intercept);

    // Since the ball is not moving, the only way to intercept it is to move to where
    // it is (at the origin)
    auto [intercept_pos, duration_until_intercept] = *best_intercept;
    EXPECT_DOUBLE_EQ(7, intercept_pos.y());
    EXPECT_DOUBLE_EQ(13, intercept_pos.y());

}

TEST(PassEvaluationTest, findBestInterceptForBall_ball_moving_to_fast_to_intercept){
    // Test where we cannot get to the ball before it leaves the field, but if we ignore
    // field boundaries we could eventually catch it
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball({0,2.9}, {0,1}, Timestamp::fromSeconds(0));
    Robot robot(0, {0,2.8}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We don't expect to be able to find an intercept
    auto best_intercept = findBestInterceptForBall(ball, field, robot);
    ASSERT_FALSE(best_intercept);
}
