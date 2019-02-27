/**
 * This file contains the unit tests for evaluation functions
 * in detect_threat.cpp
 */

#include "ai/hl/stp/evaluation/detect_threat.h"

#include <gtest/gtest.h>

#include "ai/world/ball.h"
#include "test/test_util/test_util.h"

TEST(evaluation_detect_threat_test, ball_threat_ball_vel_to_friendly_net)
{
    Vector velocity(-1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(40.2);

    Ball ball(position, velocity, timestamp);

    Field field = ::Test::TestUtil::createSSLDivBField();

    std::optional<Point> intersection =
        Evaluation::calcBallVelIntersectFriendlyNet(ball, field);


    EXPECT_EQ(Point(-4.5, -0.145), *intersection);
}

TEST(evaluation_detect_threat_test, ball_threat_ball_vel_not_to_friendly_net)
{
    Vector velocity(1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(40.2);

    Ball ball(position, velocity, timestamp);

    Field field = ::Test::TestUtil::createSSLDivBField();

    std::optional<Point> intersection =
        Evaluation::calcBallVelIntersectFriendlyNet(ball, field);


    EXPECT_EQ(std::nullopt, intersection);
}

TEST(evaluation_detect_threat_test, ball_threat_ball_vel_to_enemy_net)
{
    Vector velocity(1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(40.2);

    Ball ball(position, velocity, timestamp);

    Field field = ::Test::TestUtil::createSSLDivBField();

    std::optional<Point> intersection =
        Evaluation::calcBallVelIntersectEnemyNet(ball, field);


    EXPECT_EQ(Point(4.5, -0.165), *intersection);
}

TEST(evaluation_detect_threat_test, ball_threat_ball_vel_not_to_enemy_net)
{
    Vector velocity(-1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(40.2);

    Ball ball(position, velocity, timestamp);

    Field field = ::Test::TestUtil::createSSLDivBField();

    std::optional<Point> intersection =
        Evaluation::calcBallVelIntersectEnemyNet(ball, field);


    EXPECT_EQ(std::nullopt, intersection);
}


int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
