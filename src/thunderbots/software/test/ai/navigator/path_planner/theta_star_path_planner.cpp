#include "ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "../shared/constants.h"
#include "geom/point.h"
#include "test/test_util/test_util.h"

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_src)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Point start{2, 2}, dest{-2, -2};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(2.0, 2.0), Vector(0.0, 0.0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    obstacles.push_back(Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 1.2));

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);
    EXPECT_TRUE(path_points != std::nullopt);
    if (path_points != std::nullopt)
    {
        EXPECT_EQ(start, (*path_points)[0]);
        EXPECT_EQ(dest, (*path_points)[(*path_points).size() - 1]);
//        printf("\nThe Path is ");
//        for (Point p : *path_points)
//        {
//            {
//                printf("-> (%lf,%lf) ", p.x(), p.y());
//            }
//        }
//        printf("\n");
    }
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_dest)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Point start{0, 0}, dest{2, 2};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(2.0, 2.0), Vector(0.0, 0.0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    obstacles.push_back(Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 1.2));

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);
    EXPECT_TRUE(path_points != std::nullopt);
    if (path_points != std::nullopt)
    {
        EXPECT_EQ(start, (*path_points)[0]);
        EXPECT_NE(dest, (*path_points)[(*path_points).size() - 1]);
//        printf("\nThe Path is ");
//        for (Point p : *path_points)
//        {
//            {
//                printf("-> (%lf,%lf) ", p.x(), p.y());
//            }
//        }
//        printf("\n");
    }
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_empty_grid)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Point start{2, 2}, dest{-3, -3};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);

    auto path_points = planner->findPath(start, dest);
    EXPECT_TRUE(path_points != std::nullopt);
    if (path_points != std::nullopt)
    {
        EXPECT_EQ(2, (*path_points).size());
        EXPECT_EQ(start, (*path_points)[0]);
        EXPECT_EQ(dest, (*path_points)[1]);
//        printf("\nThe Path is ");
//        for (Point p : *path_points)
//        {
//            {
//                printf("-> (%lf,%lf) ", p.x(), p.y());
//            }
//        }
//        printf("\n");
    }
}
