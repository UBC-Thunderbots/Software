#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/obstacle_factory.h"
#include "software/new_geom/point.h"
#include "software/test_util/test_util.h"
#include "software/world/field.h"

class TestThetaStarPathPlanner : public testing::Test
{
   public:
    TestThetaStarPathPlanner()
        : obstacle_factory(
              Util::DynamicParameters->getAIConfig()->getObstacleFactoryConfig())
    {
    }

    ObstacleFactory obstacle_factory;
};

void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                       Rectangle bounding_box)
{
    for (auto const& path_point : path_points)
    {
        EXPECT_TRUE(bounding_box.contains(path_point))
            << "Path point " << path_point << " not in bounding box {"
            << bounding_box.negXNegYCorner() << "," << bounding_box.posXPosYCorner()
            << "}";
    }
}

void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                       std::vector<ObstaclePtr> obstacles)
{
    // If the path size is 1, just need to check that the point is not within the obstacle
    if (path_points.size() == 1)
    {
        for (auto const& obstacle : obstacles)
        {
            EXPECT_FALSE(obstacle->contains(path_points[0]))
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
            EXPECT_FALSE(obstacle->intersects(path_segment))
                << "Line segment {" << path_points[i] << "," << path_points[i + 1]
                << "} intersects obstacle " << obstacle;
        }
    }
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_src)
{
    // Test where we start in an obstacle. We should find the closest edge of
    // the obstacle and start our path planning there
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our starting location
    std::vector<ObstaclePtr> obstacles = {obstacle_factory.createObstacleFromRectangle(
        Rectangle(Point(-0.5, -1), Point(0.5, 1)))};

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

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_blocked_dest)
{
    // Test where we try to end in an obstacle. We should navigate to the closest point
    // on the edge of the destination
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{2.7, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {obstacle_factory.createObstacleFromRectangle(
        Rectangle(Point(2.5, -1), Point(3.5, 1)))};

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

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_x_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {obstacle_factory.createObstacleFromRectangle(
        Rectangle(Point(1, -1), Point(2, 1)))};

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

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_y_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{0, 0}, dest{0, 3};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {obstacle_factory.createObstacleFromRectangle(
        Rectangle(Point(-1, 1), Point(1, 2)))};

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

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_empty_grid)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2, 2}, dest{-3, -3};

    std::vector<ObstaclePtr> obstacles = {};

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

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_same_cell_dest)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();

    std::unique_ptr<PathPlanner> planner = std::make_unique<ThetaStarPathPlanner>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(2, path->size());
    EXPECT_EQ(start, path->startPoint());
    EXPECT_EQ(dest, path->endPoint());
}

TEST_F(TestThetaStarPathPlanner, no_navigable_area)
{
    // Test running theta star with no area to navigate in
    Point start{-1.0, -1.0}, dest{1.0, 1.0};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();
    Rectangle navigable_area({0, 0}, {1, 1});
    auto path = ThetaStarPathPlanner().findPath(start, dest, navigable_area, obstacles);

    EXPECT_EQ(std::nullopt, path);
}

// This test is disabled, it can be enabled by removing "DISABLED_" from the test name
TEST_F(TestThetaStarPathPlanner, DISABLED_performance)
{
    // This test can be used to guage performance, and profiled to find areas for
    // improvement
    std::vector<std::vector<ObstaclePtr>> obstacle_sets = {
        {
            obstacle_factory.createRobotObstacle({0, 0}),
            obstacle_factory.createRobotObstacle({0, 0.5}),
            obstacle_factory.createRobotObstacle({0, 1.0}),
            obstacle_factory.createRobotObstacle({0, 1.5}),
        },
        {
            obstacle_factory.createRobotObstacle({0, 0}),
            obstacle_factory.createRobotObstacle({0, 0.5}),
            obstacle_factory.createRobotObstacle({0, 1.0}),
            obstacle_factory.createRobotObstacle({0, 1.5}),
            obstacle_factory.createRobotObstacle({-0.5, 0}),
            obstacle_factory.createRobotObstacle({-0.5, 0.5}),
            obstacle_factory.createRobotObstacle({-0.5, 1.0}),
            obstacle_factory.createRobotObstacle({-0.5, 1.5}),
            obstacle_factory.createRobotObstacle({0.5, 0}),
            obstacle_factory.createRobotObstacle({0.5, 0.5}),
            obstacle_factory.createRobotObstacle({0.5, 1.0}),
            obstacle_factory.createRobotObstacle({0.5, 1.5}),
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
        duration / (static_cast<double>(num_iterations) * obstacle_sets.size() - 1);

    std::cout << "Took "
              << std::chrono::duration_cast<std::chrono::microseconds>(duration).count() /
                     1000.0
              << "ms to run, average time of "
              << std::chrono::duration_cast<std::chrono::microseconds>(avg).count() /
                     1000.0
              << "ms";
}
