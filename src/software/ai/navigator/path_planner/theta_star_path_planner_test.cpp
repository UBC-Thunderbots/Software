#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/new_geom/point.h"
#include "software/test_util/test_util.h"
#include "software/world/field.h"

void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                       Rectangle bounding_box)
{
    for (auto const& path_point : path_points)
    {
        EXPECT_TRUE(bounding_box.containsPoint(path_point))
            << "Path point " << path_point << " not in bounding box {"
            << bounding_box.negXNegYCorner() << "," << bounding_box.posXPosYCorner()
            << "}";
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

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({0, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    // Make sure the path does not go through any obstacles, except for the
    // first point, which is in the obstacle blocking the start position
    checkPathDoesNotIntersectObstacle({path_points.begin() + 1, path_points.end()},
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

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point
    EXPECT_EQ(start, path->startPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    // Make sure the path does not go through any obstacles
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_single_obstacle_along_x_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our destination location
    std::vector<Obstacle> obstacles = {
        Obstacle(Polygon({Point(1, 1), Point(2, 1), Point(2, -1), Point(1, -1)}))};

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 1.2}, {3.1, -1.2});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    // Can't make sure the path does not go through any obstacles
    // since start is blocked
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_single_obstacle_along_y_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{0, 3};

    // Place a rectangle over our destination location
    std::vector<Obstacle> obstacles = {
        Obstacle(Polygon({Point(1, 1), Point(1, 2), Point(-1, 2), Point(-1, 1)}))};

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box(
        {
            1.2,
            -0.1,
        },
        {-1.2, 3.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    // Can't make sure the path does not go through any obstacles
    // since start is blocked
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_empty_grid)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2, 2}, dest{-3, -3};

    std::vector<Obstacle> obstacles = {};

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    // Since there are no obstacles, there should be two path points, one at the start
    // and one at the destination
    EXPECT_EQ(2, path->size());
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());
}

TEST(TestThetaStarPathPlanner, test_theta_star_path_planner_same_cell_dest)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(2, path->size());
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());
}

TEST(TestThetaStarPathPlanner, no_navigable_area)
{
    // Test running theta star with no area to navigate in
    Point start{-1.0, -1.0}, dest{1.0, 1.0};

    std::vector<Obstacle> obstacles = std::vector<Obstacle>();
    Rectangle navigable_area({0, 0}, {0, 0});
    auto path = ThetaStarPathPlanner().findPath(start, dest, navigable_area, obstacles);

    EXPECT_EQ(std::nullopt, path);
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
                std::make_unique<ThetaStarPathPlanner>();

            Rectangle navigable_area = field.fieldBoundary();

            planner->findPath(start, dest, navigable_area, obstacles);
        }
    }

    auto end_time = std::chrono::system_clock::now();

    std::chrono::duration<double> duration = end_time - start_time;

    std::chrono::duration<double> avg =
        duration / ((double)num_iterations * obstacle_sets.size() - 1);

    //    std::cout << "Took " <<
    //    std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0
    //    << "ms to run, average time of " <<
    //    std::chrono::duration_cast<std::chrono::microseconds>(avg).count() / 1000.0 <<
    //    "ms";
}
