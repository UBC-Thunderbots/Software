#include "ai/navigator/path_planner/straight_line_path_planner.h"

#include <gtest/gtest.h>

#include "geom/point.h"

TEST(TestStraightLinePathPlanner, test_straight_line_path_planner)
{
    Point start{0, 0}, dest{1, 1};
    std::unique_ptr<PathPlanner> planner = std::make_unique<StraightLinePathPlanner>();

    auto path_points = *planner->findPath(start, dest);
    EXPECT_EQ(path_points[0], start);
    EXPECT_EQ(path_points[1], dest);
}
