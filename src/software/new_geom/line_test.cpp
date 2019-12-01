#include "software/new_geom/line.h"

#include <gtest/gtest.h>

TEST(LineTest, two_points_constructor)
{
    Line l(Point(-1.0, 2.0), Point(3.0, 4.0));
    EXPECT_EQ(l.toNormalUnitVector(), Vector(-1, 2).normalize());
}

TEST(LineTest, swapXY)
{
    Line l(Point(-1.0, 2.0), Point(3.0, 4.0));
    l.swapXY();
    EXPECT_EQ(l.toNormalUnitVector(), Vector(2, -1).normalize());
}
