#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"
#include <gtest/gtest.h>

#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

class TestEnlsvgPathPlanner : public testing::Test
{
    public:
        TestEnlsvgPathPlanner()
            : robot_navigation_obstacle_factory(
                std::make_shared<const RobotNavigationObstacleConfig>())
        {
        }
        
        RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
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
    for_each(obstacles.begin(), obstacles.end(), [](Polygon &shape) { shape = shape.expand(ROBOT_MAX_RADIUS_METERS); });
    
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

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_empty_grid)
{
    Field field = Field::createSSLDivisionAField();
    Point start{2, 2}, dest{-3, -3};
    
    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    EXPECT_EQ(2, path_points.size());
    
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_no_navigable_area)
{
    Field field = Field::createSSLDivisionAField();
    Point start{ field.enemyGoalCenter() + Vector(1, 0) }, dest{-3, -3};
    
    std::vector<ObstaclePtr> obstacles = {};
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
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
        robot_navigation_obstacle_factory.createFromShape(
            obstacle)};

    Rectangle navigable_area = field.fieldBoundary();
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    
    auto path = planner.findPath(start, dest, navigable_area, obstacles);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();
//    for (Point p : path_points)
//    {
//        std::cout << "Point (" << p.x() << ", " << p.y() << ")\n";
//    }

    // The path should start at exactly the start point and end at exactly the dest
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());

    // Make sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 1.3}, {3.1, -1.3});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, { obstacle });
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_blocked_src)
{
    // Test where we start in an obstacle. We should find the closest edge of the obstacle and start our path planning
    // there
    Field field = Field::createSSLDivisionAField();
    Point start{-2, 0}, dest{3, 3};
    
    // Place a rectangle over our starting location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(
            Rectangle(Point(-4.1, -3), Point(0, 3)))};
            
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
    
    // Make sure the path planner creates a path that exits out at the nearest edge
    // Here, the nearest edge is straight up in the x direction.
    EXPECT_GE(path_points[1].x(), 0);
    EXPECT_NEAR(0, path_points[1].y(), planner.getResolution());
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-2.1, -0.1}, {3.1, 3.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    
    // Make sure path does not go through any obstacles, except for the first point, which is in the obstacle blocking
    // the start position
    checkPathDoesNotIntersectObstacle({path_points.begin() + 1, path_points.end()}, obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_blocked_dest)
{
    // Test where we try to end in an obstacle. Make sure we navigate to the closest point on the edge of the dest
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0}, dest{2.7, 0};
    
    Polygon obstacle = Rectangle(Point(-2.5, -1), Point(3.5, 1));
    
    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};
    
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
//    for (Point p : path_points)
//    {
//        std::cout << "Point (" << p.x() << ", " << p.y() << ")\n";
//    }
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    
    // The endpoint should be the closest point to the obstacle, which is around (3.5, 0)
    EXPECT_NEAR(3.5, path->getEndPoint().x(), 0.3);
    EXPECT_NEAR(0, path->getEndPoint().y(), 0.1);
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.3, -0.3}, {3.8, 1.3});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    
    // Make sure path does not go through any obstacles, except for the first point, which is in the obstacle blocking
    // the start position
    checkPathDoesNotIntersectObstacle({path_points.begin() + 1, path_points.end()}, { obstacle });
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_same_cell_dest)
{
    // Test where we try to end in an obstacle. Make sure we navigate to the closest point on the edge of the dest
    Field field = Field::createSSLDivisionAField();
    Point start{2.29, 2.29}, dest{2.3, 2.3};

    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {};
            
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_single_obstacle_outside_of_path)
{
    // Test when there is an obstacle, but the shortest path will not intersect it
    Field field = Field::createSSLDivisionAField();
    Point start{3, 0}, dest{-3, 0};
    
    // Test an obstacle out of the path
    Polygon obstacle = Rectangle(Point(-0.27, -0.27), Point(-1, -1));
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};
            
    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Straight line path expected
    EXPECT_EQ(2, path_points.size());
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-3.1, 0.1}, {3.1, -1.0});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    
    // Make sure path does not go through any obstacles, except for the first point, which is in the obstacle blocking
    // the start position
    checkPathDoesNotIntersectObstacle({path_points.begin() + 1, path_points.end()}, { obstacle });
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
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, -1.3}, {3.1, 1.3});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
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
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({-0.1, -0.1}, {2.1, 3.1});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_not_centered_at_origin)
{
    Field field = Field::createSSLDivisionAField();
    Point start{0.5, 0.5}, dest{2.5, 2.5};
    
    Polygon obstacle = Rectangle(Point(0.7, 0), Point(2.2, 2.2));
    
    std::vector<ObstaclePtr> obstacles = {
        robot_navigation_obstacle_factory.createFromShape(obstacle)};

    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Make sure the start and end of the path are correct
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
    
    // Make the sure the path does not exceed a bounding box
    Rectangle bounding_box({0.4, 0.4}, {2.6, 2.6});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, { obstacle });
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_small_distance_slightly_greater_than_robot_radius)
{    
    Field field = Field::createSSLDivisionAField();
    Point start{0, 0}, dest{ROBOT_MAX_RADIUS_METERS+0.05, 0.01};
    
    // Place a rectangle over our destination location
    std::vector<ObstaclePtr> obstacles = {};

    Rectangle navigable_area = field.fieldBoundary();
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // Straight line path
    EXPECT_EQ(2, path_points.size());
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_EQ(dest, path->getEndPoint());
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}

TEST_F(TestEnlsvgPathPlanner, test_enlsvg_path_planner_circular_obstacle)
{
    Field field = Field::createSSLDivisionAField();
    Rectangle navigable_area = field.fieldBoundary();

    Point start{-1.4520030517578126, -1.3282811279296876},
        dest{-1.3801560736390279, -2.5909120196973863};

    std::vector<ObstaclePtr> obstacles = {std::make_shared<GeomObstacle<Circle>>(
        Circle(Point(-1.48254052734375, -2.5885056152343751), 0.27150299999999999))};
    
    EnlsvgPathPlanner planner = EnlsvgPathPlanner(navigable_area, obstacles);
    auto path = planner.findPath(start, dest, navigable_area, obstacles);
    
    EXPECT_TRUE(path != std::nullopt);
    
    std::vector<Point> path_points = path->getKnots();
    
    // since endpoint ends in obstacle, make sure the endpoint is close enough
    EXPECT_EQ(start, path->getStartPoint());
    EXPECT_LE(0.27150, distance(path->getEndPoint(), dest));
    
    // Make sure path does not exceed a bounding box
    Rectangle bounding_box({-1.46, -2.6}, {-1, -1.31});
    checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    checkPathDoesNotIntersectObstacle(path_points, obstacles);
}