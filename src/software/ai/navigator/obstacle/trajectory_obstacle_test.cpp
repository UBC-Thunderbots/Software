#include "software/ai/navigator/obstacle/trajectory_obstacle.hpp"

#include <gtest/gtest.h>


class TrajectoryObstacleTest : public testing::Test
{
public:
    TrajectoryObstacleTest() : obstacle_traj(std::make_shared<BangBangTrajectory2D>(
            start, end, initial_vel, KinematicConstraints(1, 1, 1)), BangBangTrajectory2D::generator),
            obstacle(std::make_shared<TrajectoryObstacle<Circle>>(circle, obstacle_traj))
    {
    }

    Point start = Point(0, 0);
    Point end = Point(4, 0);
    Vector initial_vel = Vector(0, 0);
    TrajectoryPath obstacle_traj;

    double radius = 1.0;
    Circle circle = Circle(start, radius);
    ObstaclePtr obstacle;
};

TEST_F(TrajectoryObstacleTest, circle_obstacle_contains)
{
    EXPECT_FALSE(obstacle->contains(Point(-radius - 0.01, 0), 0.0));
    EXPECT_TRUE(obstacle->contains(Point(-radius + 0.01, 0), 0.0));

    // 1-sec into the future, the obstacle should not be covering the point
    // that initially was barely inside it
    EXPECT_FALSE(obstacle->contains(Point(-radius + 0.01, 0), 1.0));

    // Test the obstacle once it's reached its destination
    double end_time = obstacle_traj.getTotalTime();
    EXPECT_TRUE(obstacle->contains(end, end_time));
    EXPECT_FALSE(obstacle->contains(end + Vector(radius + 0.01, 0.0), end_time));
}

TEST_F(TrajectoryObstacleTest, circle_obstacle_distance)
{
    // 0 seconds into the future
    EXPECT_EQ(obstacle->distance(Point(radius + 3, 0), 0.0), 3);
    EXPECT_EQ(obstacle->distance(Point(-radius - 3, 0), 0.0), 3);

    // Half-way along the trajectory
    double half_way = obstacle_traj.getTotalTime() / 2.0;
    Point half_way_point = obstacle_traj.getPosition(half_way);
    EXPECT_EQ(obstacle->distance(half_way_point + Vector(radius + 3, 0.0), half_way), 3);
    EXPECT_EQ(obstacle->distance(half_way_point + Vector(-radius - 3, 0.0), half_way), 3);

    // Test the distance to the obstacle once it's reached its destination
    double end_time = obstacle_traj.getTotalTime();
    EXPECT_EQ(obstacle->distance(end + Vector(radius + 3, 0.0), end_time), 3);
    EXPECT_EQ(obstacle->distance(end + Vector(-radius - 3, 0.0), end_time), 3);
}

TEST_F(TrajectoryObstacleTest, circle_obstacle_intersects)
{
    Vector offset(0, 2.0);
    Segment segment_start(start + offset, start - offset);
    Segment segment_end(end + offset, end - offset);

    // 0 seconds into the future
    EXPECT_TRUE(obstacle->intersects(segment_start, 0.0));
    EXPECT_FALSE(obstacle->intersects(segment_end, 0.0));

    // Half-way, the obstacle should have moved and not intersecting start and end segments
    double half_way = obstacle_traj.getTotalTime() / 2.0;
    Segment half_way_segment = Segment(obstacle_traj.getPosition(half_way) + offset,
                                             obstacle_traj.getPosition(half_way) - offset);
    EXPECT_TRUE(obstacle->intersects(half_way_segment, half_way));
    EXPECT_FALSE(obstacle->intersects(segment_start, half_way));
    EXPECT_FALSE(obstacle->intersects(segment_end, half_way));

    // Test the intersection once it's reached its destination
    double end_time = obstacle_traj.getTotalTime();
    EXPECT_FALSE(obstacle->intersects(segment_start, end_time));
    EXPECT_TRUE(obstacle->intersects(segment_end, end_time));
}