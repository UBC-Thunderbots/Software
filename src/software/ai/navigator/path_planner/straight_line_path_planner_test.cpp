#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"

TEST(TestStraightLinePathPlanner, test_straight_line_path_planner)
{
    Point start{0, 0}, dest{1, 1};
    std::unique_ptr<PathPlanner> planner = std::make_unique<StraightLinePathPlanner>();

    Rectangle navigable_area = Rectangle(Point(0, 0), Point(1, 1));

    auto path =
        planner->findPath(start, dest, navigable_area, std::vector<ObstaclePtr>());

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path.value();

    EXPECT_EQ(path_points.front(), start);
    EXPECT_EQ(path_points.back(), dest);
}
