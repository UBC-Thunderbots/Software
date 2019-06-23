#include "ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "../shared/constants.h"
#include "geom/point.h"
#include "test/test_util/test_util.h"

/**
 * Prints out the path formed by the points given
 * @param path_points
 */
void printPath(std::vector<Point> path_points){
        for (Point p : path_points)
        {
            {
                printf("-> (%lf,%lf) ", p.x(), p.y());
            }
        }
        printf("\n");
}

void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points, Rectangle bounding_box){
    for (auto const& path_point : path_points){
            EXPECT_TRUE(bounding_box.containsPoint(path_point));
    }
}

void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points, std::vector<Obstacle> obstacles){
    // If the path size is 1, just need to check that the point is not within the obstacle
    if (path_points.size() == 1){
        for (auto const& obstacle : obstacles){
            EXPECT_FALSE(obstacle.getBoundaryPolygon().containsPoint(path_points[0]));
        }
    }

    // Check that no line segment on the path intersects the obstacle
    for (std::size_t i = 0; i < path_points.size()-1; i++){
        Segment path_segment(path_points[i], path_points[i+1]);
        for (auto const& obstacle : obstacles){
            EXPECT_FALSE(obstacle.getBoundaryPolygon().intersects(path_segment));
        }
    }
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_src)
{
    // Test where we start in an obstacle. We should find the closest edge of
    // the obstacle and start our path planning there
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    // Place a rectangle over our starting location
    obstacles.emplace_back(Obstacle(Polygon(
            {
                    Point(0.5,1),
                    Point(-0.5,1),
                    Point(-0.5,-1),
                    Point(0.5,-1)
            }
            )));
    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);

    // We expect to find a path
    ASSERT_TRUE(path_points);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path_points->front());
    EXPECT_EQ(dest, path_points->back());

    // Make sure the path does not exceed a simple bounding box
    Rectangle bounding_box({1, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(*path_points, bounding_box);

    // Make sure the path does not go through any obstacles
    checkPathDoesNotIntersectObstacle(*path_points, obstacles);
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
