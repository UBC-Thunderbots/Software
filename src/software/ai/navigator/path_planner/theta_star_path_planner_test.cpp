#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/point.h"
#include "software/world/field.h"

class TestThetaStarPathPlanner : public testing::Test
{
   public:
    TestThetaStarPathPlanner()
        : robot_navigation_obstacle_factory(
              std::make_shared<const RobotNavigationObstacleConfig>()),
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

void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                       std::vector<Polygon> obstacles)
{
    // If the path size is 1, just need to check that the point is not within the obstacle
    if (path_points.size() == 1)
    {
        for (auto const& obstacle : obstacles)
        {
            EXPECT_FALSE(contains(obstacle, path_points[0]))
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
            EXPECT_FALSE(intersects(obstacle, path_segment))
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

    std::vector<Point> path_points = path.value();

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    // bounding box is expanded around the src because the path planner needs to find a
    // way out of the obstacle
    Rectangle bounding_box({-0.8, -1.3}, {3.1, 1.3});
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

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point
    EXPECT_EQ(start, path.value().front());

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

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 1.4}, {3.1, -1.4});
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

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 0.15}, {3.1, -1.0});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);

    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_x_axis)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{3, 0};

    // Robot navigation obstacle factory expands the obstacle by the robot radius plus a
    // constant offset. To make sure that the robot does not collide with the actual
    // obstacle we will scale the obstacle by the robot radius and check collision with
    // that.
    Polygon obstacle_shape = Rectangle(Point(1, -1), Point(2, 1));

    std::vector<ObstaclePtr> expanded_obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle_shape)};

    std::vector<Polygon> actual_obstacles = {
        obstacle_shape.expand(ROBOT_MAX_RADIUS_METERS)};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, expanded_obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 1.35}, {3.1, -1.35});

    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, actual_obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_single_obstacle_along_y_axis)
{
    // Test where we need to navigate around a single obstacle along the y-axis
    Field field = Field::createSSLDivisionBField();
    Point start{0, 0}, dest{0, 3};

    // Robot navigation obstacle factory expands the obstacle by the robot radius plus a
    // constant offset. To make sure that the robot does not collide with the actual
    // obstacle we will scale the obstacle by the robot radius and check collision with
    // that.
    Polygon obstacle_shape = Rectangle(Point(-1, 1), Point(1, 2));

    std::vector<ObstaclePtr> expanded_obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle_shape)};

    std::vector<Polygon> actual_obstacles = {
        obstacle_shape.expand(ROBOT_MAX_RADIUS_METERS)};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, expanded_obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({1.32, -0.1}, {-1.3, 3.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, actual_obstacles);
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
    EXPECT_EQ(2, path->knots.size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_same_cell_dest)
{
    Field field = Field::createSSLDivisionBField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    std::vector<ObstaclePtr> obstacles = std::vector<ObstaclePtr>();

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path.value();

    EXPECT_EQ(2, path->knots.size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
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

    std::vector<Point> path_points = path.value();

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

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

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_closest_dest_equals_penultimate_path_point)
{
    // This is a regression test for when closest dest equals the penultimate path point
    Field field = Field::createSSLDivisionBField();
    Point start{-1.4520030517578126, -1.3282811279296876},
        dest{-1.3801560736390279, -2.5909120196973863};

    std::vector<ObstaclePtr> obstacles = {std::make_shared<GeomObstacle<Circle>>(
        Circle(Point(-1.48254052734375, -2.5885056152343751), 0.27150299999999999))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    ASSERT_TRUE(path != std::nullopt);

    // Expect that the last two points are combined into one
    EXPECT_EQ(2, path->knots().size());
    EXPECT_EQ(start, path.value().front());
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_check_obstacle_edge)
{
    Field field = Field::createSSLDivisionBField();
    Point start{0, 3}, dest{3, 3};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(1, 3), Point(2, 0)))};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, dest, navigable_area, obstacles);

    ASSERT_TRUE(path != std::nullopt);

    checkPathDoesNotIntersectObstacle(path->knots(), obstacles);
}

TEST_F(TestThetaStarPathPlanner,
       test_theta_star_path_planner_straight_line_path_slightly_grazes_obstacle)
{
    Field field = Field::createSSLDivisionBField();
    Point start{1.2299999999999995, 2.0999999999999996}, end{0, 3};

    // Robot navigation obstacle factory expands the obstacle by the robot radius plus a
    // constant offset. To make sure that the robot does not collide with the actual
    // obstacle we will scale the obstacle by the robot radius and check collision with
    // that.
    Polygon obstacle_shape = Rectangle(Point(-1, 1), Point(1, 2));

    std::vector<ObstaclePtr> expanded_obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle_shape)};

    std::vector<Polygon> actual_obstacles = {
        obstacle_shape.expand(ROBOT_MAX_RADIUS_METERS)};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, end, navigable_area, expanded_obstacles);

    ASSERT_TRUE(path != std::nullopt);
    checkPathDoesNotIntersectObstacle(path->knots(), actual_obstacles);
}

TEST_F(TestThetaStarPathPlanner, test_theta_star_path_planner_start_in_enemy_half)
{
    // Start in enemy half (obstacle)
    Point start{4, 3}, end{-0.1, -2.9};
    Field field = Field::createSSLDivisionBField();

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(field.enemyHalf())};

    Rectangle navigable_area = field.fieldBoundary();

    auto path = planner->findPath(start, end, navigable_area, obstacles);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the path does not exceed a bounding box

    Rectangle bounding_box({4.5, 3.3}, {-1.3, -3.2});
    checkPathDoesNotExceedBoundingBox(path->knots(), bounding_box);
}
