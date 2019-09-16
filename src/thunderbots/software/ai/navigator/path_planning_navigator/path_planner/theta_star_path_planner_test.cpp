#include "software/ai/navigator/path_planning_navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/geom/point.h"
#include "software/test/test_util/test_util.h"

/**
 * Prints out the path formed by the points given
 * @param path_points
 */
void printPath(std::vector<Point> path_points)
{
    for (Point p : path_points)
    {
        {
            printf("-> (%lf,%lf) ", p.x(), p.y());
        }
    }
    printf("\n");
}

void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                       Rectangle bounding_box)
{
    for (auto const& path_point : path_points)
    {
        EXPECT_TRUE(bounding_box.containsPoint(path_point))
            << "Path point " << path_point << " not in bounding box {"
            << bounding_box.swCorner() << "," << bounding_box.neCorner() << "}";
    }
}

void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                       std::vector<Obstacle> obstacles)
{
    // If the path size is 1, just need to check that the point is not within the obstacle
    if (path_points.size() == 1)
    {
        for (auto const& obstacle : obstacles)
        {
            EXPECT_FALSE(obstacle.containsPoint(path_points[0]))
                << "Only point on path " << path_points[0] << " is in obstacle "
                << obstacle;
        }
    }

    // Check that no line segment on the path intersects the obstacle
    for (std::size_t i = 0; i < path_points.size() - 1; i++)
    {
        Segment path_segment(path_points[i], path_points[i + 1]);
        for (auto const& obstacle : obstacles)
        {
            EXPECT_FALSE(obstacle.intersects(path_segment))
                << "Line segment {" << path_points[i] << "," << path_points[i + 1]
                << "} intersects obstacle " << obstacle;
        }
    }
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_src)
{
    // Test where we start in an obstacle. We should find the closest edge of
    // the obstacle and start our path planning there
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our starting location
    std::vector<Obstacle> obstacles = {Obstacle(
        Polygon({Point(0.5, 1), Point(-0.5, 1), Point(-0.5, -1), Point(0.5, -1)}))};

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);

    // We expect to find a path
    ASSERT_TRUE(path_points);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path_points->front());
    EXPECT_EQ(dest, path_points->back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({0, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(*path_points, bounding_box);

    // Make sure the path does not go through any obstacles, except for the
    // first point, which is in the obstacle blocking the start position
    checkPathDoesNotIntersectObstacle({path_points->begin() + 1, path_points->end()},
                                      obstacles);
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_dest)
{
    // Test where we try to end in an obstacle. We should navigate to the closest point
    // on the edge of the destination
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{2.7, 0};

    // Place a rectangle over our destination location
    std::vector<Obstacle> obstacles = {Obstacle(
        Polygon({Point(3.5, 1), Point(2.5, 1), Point(2.5, -1), Point(3.5, -1)}))};

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);

    // We expect to find a path
    ASSERT_TRUE(path_points);

    // The path should start at exactly the start point
    EXPECT_EQ(start, path_points->front());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(*path_points, bounding_box);

    // Make sure the path does not go through any obstacles
    checkPathDoesNotIntersectObstacle(*path_points, obstacles);
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_single_obstacle_along_x_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our destination location
    std::vector<Obstacle> obstacles = {
        Obstacle(Polygon({Point(1, 1), Point(2, 1), Point(2, -1), Point(1, -1)}))};

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);

    // We expect to find a path
    ASSERT_TRUE(path_points);

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path_points->front());
    EXPECT_EQ(dest, path_points->back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 1.2}, {3.1, -1.2});
    checkPathDoesNotExceedBoundingBox(*path_points, bounding_box);

    // Can't make sure the path does not go through any obstacles
    // since start is blocked
    // checkPathDoesNotIntersectObstacle(*path_points, obstacles);
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_single_obstacle_along_y_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{0, 3};

    // Place a rectangle over our destination location
    std::vector<Obstacle> obstacles = {
        Obstacle(Polygon({Point(1, 1), Point(1, 2), Point(-1, 2), Point(-1, 1)}))};

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);


    auto path_points = planner->findPath(start, dest);

    // We expect to find a path
    ASSERT_TRUE(path_points);

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path_points->front());
    EXPECT_EQ(dest, path_points->back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box(
        {
            1.2,
            -0.1,
        },
        {-1.2, 3.1});
    checkPathDoesNotExceedBoundingBox(*path_points, bounding_box);

    // Can't make sure the path does not go through any obstacles
    // since start is blocked
    // checkPathDoesNotIntersectObstacle(*path_points, obstacles);
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_empty_grid)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2, 2}, dest{-3, -3};

    std::vector<Obstacle> obstacles = {};

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);

    auto path_points = planner->findPath(start, dest);

    // We should be able to find path points
    ASSERT_TRUE(path_points);

    // Since there are no obstacles, there should be two path points, one at the start
    // and one at the destination
    ASSERT_EQ(2, path_points->size());
    ASSERT_EQ(start, path_points->front());
    ASSERT_EQ(dest, path_points->back());
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_same_cell_dest)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    std::unique_ptr<PathPlanner> planner =
        std::make_unique<ThetaStarPathPlanner>(field, obstacles);

    auto path_points = planner->findPath(start, dest);

    ASSERT_TRUE(path_points);
    ASSERT_EQ(2, path_points->size());
    ASSERT_EQ(start, path_points->front());
    ASSERT_EQ(dest, path_points->back());
}

TEST(TestThetaStarPathPlanner, performance)
{
    // This test can be used to guage performance, and profiled to find areas for
    // improvement
    std::vector<std::vector<Obstacle>> obstacle_sets = {
        {
            Obstacle::createCircleObstacle({0, 0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 0.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 1.0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 1.5}, ROBOT_MAX_RADIUS_METERS, 1),
        },
        {
            Obstacle::createCircleObstacle({0, 0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 0.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 1.0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0, 1.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({-0.5, 0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({-0.5, 0.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({-0.5, 1.0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({-0.5, 1.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0.5, 0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0.5, 0.5}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0.5, 1.0}, ROBOT_MAX_RADIUS_METERS, 1),
            Obstacle::createCircleObstacle({0.5, 1.5}, ROBOT_MAX_RADIUS_METERS, 1),
        }};
    Field field = ::Test::TestUtil::createSSLDivBField();

    int num_iterations = 10;

    Point start(0, 0), dest(4.5, 0);

    auto start_time = std::chrono::system_clock::now();
    for (int i = 0; i < num_iterations; i++)
    {
        for (auto obstacles : obstacle_sets)
        {
            std::unique_ptr<PathPlanner> planner =
                std::make_unique<ThetaStarPathPlanner>(field, obstacles);

            auto path_points = planner->findPath(start, dest);
        }
    }

    auto end_time = std::chrono::system_clock::now();

    std::chrono::duration<double> duration = end_time - start_time;

    std::chrono::duration<double> avg =
        duration / ((double)num_iterations * obstacle_sets.size());

    //    std::cout << "Took " <<
    //    std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0
    //    << "ms to run, average time of " <<
    //    std::chrono::duration_cast<std::chrono::microseconds>(avg).count() / 1000.0 <<
    //    "ms";
}
