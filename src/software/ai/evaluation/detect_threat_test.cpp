#include "software/ai/evaluation/detect_threat.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"

// Test where the ball will intersect the friendly net
TEST(evaluation_detect_threat_test, ball_threat_ball_intersect_friendly_net)
{
    Vector velocity(-1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(20);

    Ball ball(position, velocity, timestamp);

    Field field = Field::createSSLDivisionBField();

    std::optional<Point> intersection = calcBallVelIntersectFriendlyNet(ball, field);

    EXPECT_EQ(Point(-4.5, -0.145), intersection.value());
}

// Test where the ball will not intersect the friendly net
TEST(evaluation_detect_threat_test, ball_threat_ball_not_intersect_friendly_net)
{
    Vector velocity(1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(20);

    Ball ball(position, velocity, timestamp);

    Field field = Field::createSSLDivisionBField();

    std::optional<Point> intersection = calcBallVelIntersectFriendlyNet(ball, field);


    EXPECT_EQ(std::nullopt, intersection);
}

// Test where the ball will intersect the enemy net
TEST(evaluation_detect_threat_test, ball_threat_ball_intersect_enemy_net)
{
    Vector velocity(1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(20);

    Ball ball(position, velocity, timestamp);

    Field field = Field::createSSLDivisionBField();

    std::optional<Point> intersection = calcBallVelIntersectEnemyNet(ball, field);

    EXPECT_EQ(Point(4.5, -0.165), intersection.value());
}

// Test where the ball will not intersect the enemy net
TEST(evaluation_detect_threat_test, ball_threat_ball_not_intersect_enemy_net)
{
    Vector velocity(-1, 0.01);
    Point position(1, -0.2);
    Timestamp timestamp = Timestamp::fromSeconds(20);

    Ball ball(position, velocity, timestamp);

    Field field = Field::createSSLDivisionBField();

    std::optional<Point> intersection = calcBallVelIntersectEnemyNet(ball, field);


    EXPECT_EQ(std::nullopt, intersection);
}
