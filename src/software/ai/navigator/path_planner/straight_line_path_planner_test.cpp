#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"

TEST(TestStraightLinePathPlanner, test_straight_line_path_planner)
{
    Point start{0, 0}, dest{1, 1};
    std::unique_ptr<PathPlanner> planner = std::make_unique<StraightLinePathPlanner>();

    Rectangle navigableArea = Rectangle(Point(0, 0), 0, 0);

    Path path = planner->findPath(start, dest, navigableArea, std::vector<Obstacle>());

    EXPECT_EQ(path.calculateValue(0), start);
    EXPECT_EQ(path.calculateValue(1), dest);
}
