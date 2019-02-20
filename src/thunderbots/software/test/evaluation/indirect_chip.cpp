/**
 * Unit tests for indirect_chip_and_chase_target evaluation function and its related
 * functions.
 */
#include "ai/hl/stp/evaluation/indirect_chip.h"

#include <gtest/gtest.h>

#include "ai/world/world.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/util.h"
#include "test/test_util/test_util.h"

TEST(GetTriangleCenterAndAreaTest, get_triangle_center_and_area_test)
{
    Triangle triangle = {Point(-1, -1), Point(0, sqrt(0.75)), Point(1, -1)};
    Point p1          = triangle[0];
    Point p2          = triangle[1];
    Point p3          = triangle[2];

    Point center = Point(0, (-1 + sqrt(0.75) - 1) / 3);
    double area  = abs(0.5 * (2 * 1));

    std::pair<Point, double> test_pair = std::make_pair(center, area);
    EXPECT_EQ(test_pair, Evaluation::get_triangle_center_and_area(triangle));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
