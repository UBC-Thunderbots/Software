#include "software/geom/triangle.h"

#include <gtest/gtest.h>

TEST(TriangleConstructorTests, test_right_triangle)
{
    Triangle t = Triangle(Point(0, 0), Point(0, 5), Point(10, 0));
    EXPECT_EQ(t.area(), 25);
}

TEST(TriangleConstructorTests, test_scalene_triangle)
{
    Triangle t = Triangle(Point(0, 0), Point(-2, 6), Point(15, 0));
    EXPECT_EQ(t.area(), 45);
}

TEST(TriangleConstructorTests, test_isosceles_triangle)
{
    Triangle t = Triangle(Point(0, 0), Point(5, 0), Point(2.5, 7));
    EXPECT_EQ(t.area(), 17.5);
}

TEST(TriangleCenterTests, test_center_right)
{
    Triangle t = Triangle(Point(0, 0), Point(0, 6), Point(9, 0));
    EXPECT_EQ(t.mean(), Point(3, 2));
}

TEST(TriangleCenterTests, test_center_scalene)
{
    Triangle t = Triangle(Point(0, 0), Point(-2, 6), Point(14, 0));
    EXPECT_EQ(t.mean(), Point(4, 2));
}

TEST(TriangleCenterTests, test_center_isosceles)
{
    Triangle t = Triangle(Point(0, 0), Point(6, 0), Point(3, 12));
    EXPECT_EQ(t.mean(), Point(3, 4));
}
