#include "software/geom/spline.h"

#include <gtest/gtest.h>

TEST(TestSpline, test_spline_points_constructor)
{
    std::vector<Point> points({Point(4, 18), Point(3, 0), Point(-7, -1)});
    Spline s(points);
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.valueAt(1.0), points[1]);
    EXPECT_EQ(s.valueAt(2.0), points[2]);
}

TEST(TestSpline, test_polynomial_points_list_constructor)
{
    std::vector<Point> points({Point(1, 2), Point(2, 3), Point(0, -1)});
    Spline s({Point(1, 2), Point(2, 3), Point(0, -1)});
    EXPECT_EQ(s.size(), points.size());
    EXPECT_EQ(s.valueAt(0.0), points[0]);
    EXPECT_EQ(s.valueAt(1.0), points[1]);
    EXPECT_EQ(s.valueAt(2.0), points[2]);
}
