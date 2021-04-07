#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/point.h"
#include "software/test_util/test_util.h"
#include "software/world/field.h"

class TestThetaStarPathPlanner : public testing::Test
{
   public:
    TestThetaStarPathPlanner()
        : robot_navigation_obstacle_factory(
              std::make_shared<const RobotNavigationObstacleFactoryConfig>()),
          planner(std::make_unique<ThetaStarPathPlanner>())
    {
    }

    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    std::unique_ptr<PathPlanner> planner;
};

void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                       Rectangle bounding_box)
{
    for (auto const& path_point : path_points)
    {
        EXPECT_TRUE(contains(bounding_box, path_point))
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
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our starting location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-0.5, -1), Point(0.5, 1)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    // bounding box is expanded around the src because the path planner needs to find a
    // way out of the obstacle
    Rectangle bounding_box({-1, 1.3}, {3.1, -0.1});
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
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{2.7, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(2.5, -1), Point(3.5, 1)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point
    EXPECT_EQ(start, path->getStartPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 0.1}, {3.1, -0.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    // Make sure the path does not go through any obstacles
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_to_navigate_around)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{-3, 0}, dest{3, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-1, -1), Point(1, 1)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 1.3}, {3.1, -1.3});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_outside_of_path)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{3, 0}, dest{-3, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-0.2, -0.2), Point(-1, -1)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 0.05}, {3.1, -1.0});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_x_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{3, 0};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(1, -1), Point(2, 1)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 1.3}, {3.1, -1.3});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_y_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{0, 3};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-1, 1), Point(1, 2)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({1.3, -0.1}, {-1.3, 3.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_empty_grid)
{
    Field field = Field::createSSLDivisionBField();
    Point start{2, 2}, dest{-3, -3};

    std::vector<ObstaclePtr> obstacles = {};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    // Since there are no obstacles, there should be two path points, one at the start
    // and one at the destination
    EXPECT_EQ(2, path->getNumKnots());
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_same_cell_dest)
{
    Field field = Field::createSSLDivisionBField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(2, path->getNumKnots());
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
}

TEST_F(TestThetaStarPathPlanner, no_navigable_area)
{
    // Test running theta star with no area to navigate in
    Point start{-1.0, -1.0}, dest{1.0, 1.0};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();
    Rectangle navigable_area({0, 0}, {1, 1});
    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_EQ(std::nullopt, path);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_navigable_area_not_centred_at_origin)
{
    Point start{0.5, 0.5}, dest{2.5, 2.5};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();

    Rectangle navigable_area{{0.0, 0.0}, {3.0, 3.0}};

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed the navigable area
    checkPathDoesNotExceedBoundingBox(path_points, navigable_area);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_small_distance_that_is_greater_than_robot_radius)
{
    Field field = Field::createSSLDivisionBField();
    Point start{-4.528, -3.119}, dest{-4.44, -3.031};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    ASSERT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_robot_in_obstacle)
{
    Field field = Field::createSSLDivisionBField();
    Point start{1, 0}, dest{1, 0};
    //    Point start{0.96905413818359376, -0.26277551269531252}, dest{1,0};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();
    obstacles.emplace_back(
        std::make_shared<GeomObstacle<Circle>>(Circle(Point(1, 0), 0.27150299999999999)));

    Rectangle navigable_area = field.fieldBoundary();

    auto start_time    = std::chrono::system_clock::now();
    auto path          = planner->findPath(start, dest, navigable_area, obstacles);
    double duration_ms = ::TestUtil::millisecondsSince(start_time);
    std::cout << duration_ms << std::endl;

    ASSERT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());  // should be in an unblocked point
}
