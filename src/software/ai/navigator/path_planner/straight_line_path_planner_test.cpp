#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"

TEST(TestStraightLinePathPlanner, test_straight_line_path_planner)
{
    std::vector<Point> path_points;
    Point start{0, 0}, dest{1, 1};
    std::unique_ptr<PathPlanner> planner = std::make_unique<StraightLinePathPlanner>();

    try
    {
        path_points = std::get<std::vector<Point>>(planner->findPath(
            start, dest, Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0)),
            std::vector<Obstacle>()));
    }
    catch (const std::bad_variant_access&)
    {
        FAIL();
    }

    EXPECT_EQ(path_points[0], start);
    EXPECT_EQ(path_points[1], dest);
}
