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

    // We expect to receive a pass
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

// Test where we cannot get to the ball ever (even ignoring the field boundaries)

// Test where we cannot get to the ball before it leaves the field, but if we ignore
// field boundaries we could eventually catch it