#include "software/geom/algorithms/calculate_block_cone.h"

#include <gtest/gtest.h>

TEST(CalculateBlockConeTest, test_calc_block_cone)
{
    Point a(5, 10);
    Point b(-5, 10);
    Point o(0, 0);

    EXPECT_TRUE((calculateBlockCone(a, b, o, 1) - Point(0, sqrt(5))).length() < 0.00001);

    a = Point(6, 11);
    b = Point(-4, 11);
    o = Point(1, 1);

    EXPECT_TRUE((Point(calculateBlockCone(a, b, o, 1) - Point(0, sqrt(5))) - o).length() <
                0.00001);

    a = Point(-2, 6);
    b = Point(2, 2);
    o = Point(-2, -2);

    EXPECT_TRUE(
        (Point(calculateBlockCone(a, b, o, 1) - Point(1, 1.0 + sqrt(2))) - o).length() <
        0.0001);
}
