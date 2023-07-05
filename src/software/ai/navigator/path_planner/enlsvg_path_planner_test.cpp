#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"

#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/test_util/path_planning_test_util.h"
#include "software/test_util/test_util.h"

class TestEnlsvgPathPlanner : public testing::Test
{
   public:
    TestEnlsvgPathPlanner()
        : robot_navigation_obstacle_config(TbotsProto::RobotNavigationObstacleConfig()),
          robot_navigation_obstacle_factory(robot_navigation_obstacle_config)
    {
    }

    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
};

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_empty_grid)
{
    Field field = Field::createSSLDivisionAField();
    Point start{2, 2}, dest{-3, -3};

    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area           = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    EXPECT_EQ(2, path.value().size());

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_no_navigable_area)
{
    Field field = Field::createSSLDivisionAField();
    Point start{field.enemyGoalCenter() + Vector(1, 0)}, dest{-3, -3};

    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area           = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path == std::nullopt);
}

TEST_F(TestEnlsvgPathPlanner,
       test_enlsvg_star_path_planner_single_obstacle_to_navigate_around)
{
    // Test where we need to navigate around a single obstacle along the x-axis
    Field field = Field::createSSLDivisionBField();
    Point start{-3, 0}, dest{3, 0};

    Polygon obstacle = Rectangle(Point(-1, -1), Point(1, 1));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());

    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 1.3}, {3.1, -1.3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), {obstacle});
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_blocked_src)
{
    // Test where we start in an obstacle. We should find the closest edge of the obstacle
    // and start our path planning there
    Field field = Field::createSSLDivisionAField();
    Point start{-2, 0}, dest{3, 3};

    // Place a rectangle over our starting location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-4.1, -3), Point(0, 3)))};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure the path planner creates a path that exits out at the nearest edge
    // Here, the nearest edge is straight up in the x direction.
    EXPECT_GE(path.value()[1].x(), 0);
    EXPECT_NEAR(0, path.value()[1].y(), planner.getResolution());

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-2.1, -0.1}, {3.1, 3.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);

    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_blocked_dest)
{
    // Test where we try to end in an obstacle. Make sure we navigate to the closest point
    // on the edge of the dest
    Field field = Field::createSSLDivisionAField();
    Point start{0, 2}, dest{2.7, 0};

    Polygon obstacle = Rectangle(Point(-2.5, -1), Point(3.5, 1));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());

    // The endpoint should be the closest point to the obstacle, which is around (3.5, 0)
    EXPECT_NEAR(3.5, path.value().back().x(), 0.3);
    EXPECT_NEAR(0, path.value().back().y(), 0.1);

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.3, -1.1}, {3.8, 2.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);

    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, {obstacle});
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_blocked_dest_blocked_src)
{
    // Test where we start and end in an obstacle
    // We expect to exit out of the obstacle and path plan to a point next to the final
    // point outside the obstacle
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0.1}, dest{2.7, 0};

    Polygon obstacle = Rectangle(Point(-2, -1.5), Point(3.5, 1));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());

    // The point following the start should be the closest point out of the obstacle,
    // which is around (0, 1)
    EXPECT_NEAR(0, path.value()[1].x(), 0.3);
    EXPECT_NEAR(1, path.value()[1].y(), 0.3);

    // The endpoint should be the closest point to the obstacle, which is around (3.5, 0)
    EXPECT_NEAR(3.5, path.value().back().x(), 0.3);
    EXPECT_NEAR(0, path.value().back().y(), 0.1);

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.3, -1.1}, {3.8, 2.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);

    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, {obstacle});
}

TEST_F(TestEnlsvgPathPlanner,
       test_enlsvg_path_planner_blocked_src_move_away_from_target_to_get_to_final_dest)
{
    // Test where we start inside an obstacle, but we need to briefly move away from the
    // target point to get to the final destination
    Field field = Field::createSSLDivisionAField();
    Point start{1, 2}, dest{5, 4};

    Polygon obstacle = Rectangle(Point(0, 1), Point(4, 4));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // The point following the start should be the closest point out of the obstacle,
    // which is around (0 , 2)
    EXPECT_NEAR(0, path.value()[1].x(), 0.3);
    EXPECT_NEAR(2, path.value()[1].y(), 0.3);

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.3, 0.9}, {5.1, 5.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);

    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, {obstacle});
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_same_cell_dest)
{
    // Test where we try to end in an obstacle. Make sure we navigate to the closest point
    // on the edge of the dest
    Field field = Field::createSSLDivisionAField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_single_obstacle_outside_of_path)
{
    // Test when there is an obstacle, but the shortest path will not intersect it
    Field field = Field::createSSLDivisionAField();
    Point start{3, 0}, dest{-3, 0};

    // Test an obstacle out of the path
    Polygon obstacle                   = Rectangle(Point(-0.27, -0.27), Point(-1, -1));
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Straight line path expected
    EXPECT_EQ(2, path.value().size());

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 0.1}, {3.1, -1.0});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);

    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, {obstacle});
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_single_obstacle_along_x_axis)
{
    // Test single obstacle along x-axis
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0}, dest{3, 0};

    Polygon obstacle = Rectangle(Point(1, -1), Point(2, 1));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, -1.3}, {3.1, 1.3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_single_obstacle_along_y_axis)
{
    // Test single obstacle along y-axis
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0}, dest{0, 3};

    Polygon obstacle = Rectangle(Point(1, -1), Point(2, 1));

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, -0.1}, {2.1, 3.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_not_centered_at_origin)
{
    Field field = Field::createSSLDivisionAField();
    Point start{0.5, 0.5}, dest{2.5, 2.5};

    Polygon obstacle = Rectangle(Point(0.7, 0), Point(2.2, 2.2));

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({0.4, 0.4}, {2.6, 2.6});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), {obstacle});
}

TEST_F(TestEnlsvgPathPlanner,
       test_enlsvg_path_planner_small_distance_slightly_greater_than_robot_radius)
{
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0}, dest{ROBOT_MAX_RADIUS_METERS + 0.05, 0.01};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {};

    Rectangle navigable_area = field.fieldBoundary();

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // Straight line path
    EXPECT_EQ(2, path.value().size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_circular_obstacle)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{-1.4520030517578126, -1.3282811279296876},
        dest{-1.3801560736390279, -2.5909120196973863};

    std::vector<ObstaclePtr> obstacles = {std::make_shared<GeomObstacle<Circle>>(
        Circle(Point(-1.48254052734375, -2.5885056152343751), 0.27150299999999999))};

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    // since endpoint ends in obstacle, make sure the endpoint is close enough
    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(0.27150, distance(path.value().back(), dest));

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-1.46, -2.6}, {-1, -1.31});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enslvg_path_planner_check_obstacle_edge)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{0, 3}, dest{3, 3};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(1, 3), Point(2, 0)))};

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-0.1, 2.9}, {3.1, 3.3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner,
       test_enslvg_path_planner_straight_line_path_slightly_grazes_obstacle)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{1.2299999999999995, 2.0999999999999996}, dest{0, 3};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-1, 1), Point(1, 2)))};

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({1.3, 2}, {-0.1, 3.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enslvg_path_planner_start_in_enemy_half)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    // start in enemy half (obstacle)
    Point start{4, 3}, dest{-0.1, -2.9};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(field.enemyHalf()),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());

    // Ensure path planner exits out the closest point outside the enemy half
    EXPECT_LE(path.value()[1].x(), 0);
    EXPECT_NEAR(start.y(), path.value()[1].y(), 0.1);

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({4.1, 3.1}, {-0.3, -3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
}

TEST_F(TestEnlsvgPathPlanner, test_start_in_boundary_margin)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    // start in top left corner in the boundary zone
    Point start = field.enemyCornerPos() + Vector(0.2, 0.2);
    Point dest{-1, -2};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(0, 0), Point(3, 3))),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure second point is not in the margins, but in the playing field and is close
    // to the start point
    EXPECT_TRUE(contains(field.fieldLines(), path.value()[1]));
    EXPECT_LE(0.3, distance(start, path.value()[1]));

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-1.1, -2.1}, {6.3, 4.8});

    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_one_path_planner_object_called_twice_for_same_path)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{-3, -4}, dest{2, 3};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-1.5, -1), Point(2, 1))),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-3.1, -4.1}, {2.1, 3.1});

    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);

    // Test same path using the same path planner object to make sure it still works when
    // being reused
    auto same_path                      = planner.findPath(start, dest);
    std::vector<Point> same_path_points = path.value();

    for (unsigned i = 0; i < path.value().size(); ++i)
    {
        EXPECT_EQ(path.value()[i], same_path_points[i]);
    }
}

TEST_F(TestEnlsvgPathPlanner,
       test_one_path_planner_object_called_twice_for_different_path)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{-2, -1}, dest{4, 4};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(0, -1), Point(3, 1.5))),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    std::optional<Path> path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-2.1, -1.1}, {4.1, 4.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);

    // Make sure that reusing the path planner still works by computing another path
    start = Point(5, 0);
    dest  = Point(-2, 0.5);

    auto path_two = planner.findPath(start, dest);

    ASSERT_TRUE(path_two != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path_two.value().front());
    EXPECT_EQ(dest, path_two.value().back());

    EXPECT_LE(4, path_two.value().size());

    // Make sure path does not exceed a bounding box;
    bounding_box = Rectangle(Point(5.1, 1.8), Point(-2.1, -0.1));
    TestUtil::checkPathDoesNotExceedBoundingBox(path_two.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path_two.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_going_around_defense_area)
{
    Field field              = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{4.13, -1.97}, dest{2.02, 1.96};

    std::vector<Polygon> obstacle_polygons{
        Rectangle(Point(3.018497, -1.481503), Point(4.8, 1.481503)),
    };

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(3.018497, -1.481503), Point(4.8, 1.481503))),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    std::optional<Path> path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({1, -2.5}, {5, 5});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacle_polygons);
}

// TODO (#2913): ENLSVG can not find a path from above defense area to below
TEST_F(TestEnlsvgPathPlanner, DISABLED_test_going_from_above_enemy_defense_area_to_below)
{
    World world = ::TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_B);
    const Field& field       = world.field();
    Rectangle navigable_area = field.fieldBoundary();

    // Above defense area
    Point start{4, 2};
    // Below defense area
    Point dest{4, -2};

    std::vector<ObstaclePtr> obstacles =
        robot_navigation_obstacle_factory.createFromMotionConstraint(
            TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, world);

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, ROBOT_MAX_RADIUS_METERS);
    std::optional<Path> path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({3, 3}, {4.5, -3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_going_from_below_enemy_defense_area_to_above)
{
    World world = ::TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_B);
    const Field& field       = world.field();
    Rectangle navigable_area = field.fieldBoundary();

    // Below defense area
    Point start{4, -2};
    // Above defense area
    Point dest{4, 2};

    // Inflated defense area that robots try to avoid
    std::vector<ObstaclePtr> inflated_defense_area =
        robot_navigation_obstacle_factory.createFromMotionConstraint(
            TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, world);
    // Actual enemy defense area which we must avoid going through
    std::vector<ObstaclePtr> defense_area =
        robot_navigation_obstacle_factory.createFromMotionConstraint(
            TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA, world);

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, inflated_defense_area, ROBOT_MAX_RADIUS_METERS);
    std::optional<Path> path = planner.findPath(start, dest);

    ASSERT_TRUE(path != std::nullopt);

    // Make sure the start and end points match
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({3, 3}, {4.5, -3});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), defense_area);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_speed_test)
{
    // This test does not assert anything. It prints how long it takes to path plan 121
    // pseudo-randomly generated paths once the planner is initialized
    const int num_paths_to_gen{121};

    // Create blank div A field
    Field field        = Field::createSSLDivisionAField();
    Team friendly_team = Team(Duration::fromMilliseconds(1000));
    Team enemy_team    = Team(Duration::fromMilliseconds(1000));
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));
    World world        = World(field, ball, friendly_team, enemy_team);

    Rectangle navigable_area = world.field().fieldBoundary();

    std::mt19937 random_num_gen;
    std::uniform_real_distribution x_distribution(-world.field().xLength() / 2,
                                                  world.field().xLength() / 2);
    std::uniform_real_distribution y_distribution(-world.field().yLength() / 2,
                                                  world.field().yLength() / 2);

    // Create static obstacles with friendly and enemy defense areas blocked off along
    // with the centre circle
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::CENTER_CIRCLE,
             TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
             TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA},
            world.field()),
    };

    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, world.field().boundaryMargin());

    auto start_time = std::chrono::system_clock::now();

    for (int i = 0; i < num_paths_to_gen; ++i)
    {
        Point start{x_distribution(random_num_gen), y_distribution(random_num_gen)};
        Point end{x_distribution(random_num_gen), y_distribution(random_num_gen)};

        auto path = planner.findPath(start, end);
    }

    double duration_ms = ::TestUtil::millisecondsSince(start_time);
    double avg_ms      = duration_ms / static_cast<double>(num_paths_to_gen);

    std::cout << "Took " << duration_ms << "ms to run, average time of " << avg_ms << "ms"
              << std::endl;
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_close_start_end)
{
    Field field = Field::createSSLDivisionBField();
    Point start{4.39, -2.88}, dest{4.37, -2.86};
    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area           = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);
    EXPECT_EQ(2, path.value().size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_close_start_end_but_blocked)
{
    Field field = Field::createSSLDivisionBField();
    Point start{4.39, -2.88}, dest{4.37, -2.86};
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(4.36, -2.85), Point(4.4, -2.89))),
    };
    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(0.1, distance(path.value().back(), dest));

    // Make sure path does not exceed a bounding box;
    Rectangle bounding_box = Rectangle(Point(4.3, -2.57), Point(4.5, -2.9));
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    // Make sure path does not go through any obstacles, except for the first point, which
    // is in the obstacle blocking the start position
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path.value().begin() + 1, path.value().end()}, obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_simulated_hrvo)
{
    Field field = Field::createSSLDivisionBField();

    Point start{2.90502, 0.0315793}, dest{3, 0};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA}, field),
    };
    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());

    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(2, path.value().size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(distance(path.value().back(), dest), 0.1);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_defense_play)
{
    Field field = Field::createSSLDivisionBField();

    Point start{-3.24629, 0.892211}, dest{-3.3185, 0.891779};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
             TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA},
            field),
    };
    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(2, path.value().size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(distance(path.value().back(), dest), 0.1);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_enemy_ball_placement)
{
    Field field = Field::createSSLDivisionBField();

    Point start{-3.22959, -0.675264}, dest{-3.3184, -0.695291};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
             TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE},
            field),
    };
    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(2, path.value().size());
    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(distance(path.value().back(), dest), 0.1);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_friendly_goal_motion_constraint)
{
    // Simulate a path where the goalie is slightly inside the friendly goal and will need
    // to path plan to the corner of the field.
    Field field = Field::createSSLDivisionBField();

    // Goalie inside the friendly goal touching the ball wall going to the corner
    Point start{-4.59, 0}, dest{-4.6, -3};

    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::FRIENDLY_GOAL}, field),
    };
    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner =
        EnlsvgPathPlanner(navigable_area, obstacles, field.boundaryMargin());
    auto path = planner.findPath(start, dest);
    EXPECT_TRUE(path != std::nullopt);

    // Check that the robot goes around the net when going from inside the net to a corner
    // on friendly side of the field
    TestUtil::checkPathDoesNotIntersectObstacle(path.value(), obstacles);
}
