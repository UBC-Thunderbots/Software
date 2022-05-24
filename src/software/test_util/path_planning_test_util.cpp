#include "software/test_util/path_planning_test_util.h"

void TestUtil::checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                                 std::vector<Polygon> obstacles)
{
    for_each(obstacles.begin(), obstacles.end(),
             [](Polygon& shape) { shape = shape.expand(ROBOT_MAX_RADIUS_METERS); });

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

void TestUtil::checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
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


void TestUtil::checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
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

void TestUtil::printPathPoints(std::vector<Point> path_points)
{
    std::cout << "Proposed path is: ";
    for (Point& p : std::vector(path_points.begin(), path_points.end() - 1))
    {
        std::cout << "(" << p.x() << ", " << p.y() << "), ";
    }
    std::cout << path_points.back() << "\n";
}
