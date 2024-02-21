#include "software/ai/navigator/obstacle/const_velocity_obstacle.hpp"

#include <gtest/gtest.h>
#include <math.h>

TEST(ConstVelocityObstacleTest, circle_obstacle_contains)
{
    Circle circle({2, 2}, 1);
    ObstaclePtr obstacle(std::make_shared<ConstVelocityObstacle<Circle>>(circle, Vector(0.5, 0), 1.0));
    // Point barely inside obstacle
    Point point_1(1.01, 2);
    // Point barely outside obstacle
    Point point_2(3.01, 2);
    
    EXPECT_TRUE(obstacle->contains(point_1));
    EXPECT_FALSE(obstacle->contains(point_2));

    // After 1 second, the obstacle should have moved to (2.5, 2)
    EXPECT_FALSE(obstacle->contains(point_1, 1.0));
    EXPECT_TRUE(obstacle->contains(point_2, 1.0));
    
    // After >1 seconds, we should not predict the obstacles position
    EXPECT_FALSE(obstacle->contains(point_1, 1.0));
    EXPECT_TRUE(obstacle->contains(point_2, 1.0));
}

TEST(ConstVelocityObstacleTest, circle_obstacle_distance)
{
    Circle circle({0, 0}, 1.0);
    ObstaclePtr obstacle(std::make_shared<ConstVelocityObstacle<Circle>>(circle, Vector(1.0, 0), 1.0));
    Point point_1(3.0, 0.0);
    Point point_2(2.0, 0.0);

    // 0 seconds into the future
    EXPECT_EQ(obstacle->distance(point_1, 0.0), 2.0);
    EXPECT_EQ(obstacle->distance(point_2, 0.0), 1.0);

    // 1 second into the future
    EXPECT_EQ(obstacle->distance(point_1, 1.0), 1.0);
    EXPECT_EQ(obstacle->distance(point_2, 1.0), 0.0);
    
    // After >1 seconds, we should not predict the obstacles position
    EXPECT_EQ(obstacle->distance(point_1, 2.0), 1.0);
    EXPECT_EQ(obstacle->distance(point_2, 2.0), 0.0);
}

TEST(ConstVelocityObstacleTest, circle_obstacle_intersects)
{
    Circle circle({0, 0}, 1);
    ObstaclePtr obstacle(std::make_shared<ConstVelocityObstacle<Circle>>(circle, Vector(1.0, 0), 3.0));

    Segment segment_1(Point(0.0, -5.0), Point(0.0, 5.0));
    Segment segment_2(Point(3.0, -5.0), Point(3.0, 5.0));

    // 0 seconds into the future
    EXPECT_TRUE(obstacle->intersects(segment_1, 0.0));
    EXPECT_FALSE(obstacle->intersects(segment_2, 0.0));

    // 3 seconds into the future
    EXPECT_FALSE(obstacle->intersects(segment_1, 3.0));
    EXPECT_TRUE(obstacle->intersects(segment_2, 3.0));

    // After >3 seconds, we should not predict the obstacles position
    EXPECT_FALSE(obstacle->intersects(segment_1, 3.0));
    EXPECT_TRUE(obstacle->intersects(segment_2, 3.0));
}