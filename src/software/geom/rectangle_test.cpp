#include "software/geom/rectangle.h"

#include <gtest/gtest.h>

TEST(RectangleConstructorTests, test_bottom_left_top_right)
{
    Rectangle r = Rectangle(Point(0, 0), Point(4, 5));
    EXPECT_EQ(r.xLength(), 4);
    EXPECT_EQ(r.yLength(), 5);
}

TEST(RectangleConstructorTests, test_top_left_bottom_right)
{
    Rectangle r = Rectangle(Point(-3, 3), Point(4, -5));
    EXPECT_EQ(r.xLength(), 7);
    EXPECT_EQ(r.yLength(), 8);
}

TEST(RectangleConstructorTests, test_top_right_bottom_left)
{
    Rectangle r = Rectangle(Point(2, 7), Point(0, 0));
    EXPECT_EQ(r.xLength(), 2);
    EXPECT_EQ(r.yLength(), 7);
}

TEST(RectangleConstructorTests, test_bottom_right_top_left)
{
    Rectangle r = Rectangle(Point(2, -2), Point(-3, 5));
    EXPECT_EQ(r.xLength(), 5);
    EXPECT_EQ(r.yLength(), 7);
}

TEST(RectangleAreaTests, test_positive_points)
{
    Rectangle r = Rectangle(Point(0, 0), Point(7, 8));
    EXPECT_EQ(r.area(), 56);
}

TEST(RectangleAreaTests, test_negative_points)
{
    Rectangle r = Rectangle(Point(-5, -5), Point(-1, -1));
    EXPECT_EQ(r.area(), 16);
}

TEST(RectangleAreaTests, test_positive_and_negative_points)
{
    Rectangle r = Rectangle(Point(-4, -3), Point(1, 2));
    EXPECT_EQ(r.area(), 25);
}

TEST(RectangleCentreTests, test_positive_points)
{
    Rectangle r = Rectangle(Point(0, 0), Point(8, 8));
    EXPECT_EQ(r.centre(), Point(4, 4));
}

TEST(RectangleCentreTests, test_negative_points)
{
    Rectangle r = Rectangle(Point(-1, -1), Point(-8, -8));
    EXPECT_EQ(r.centre(), Point(-4.5, -4.5));
}

TEST(RectangleCentreTests, test_positive_and_negative_points)
{
    Rectangle r = Rectangle(Point(-4, -3), Point(1, 2));
    EXPECT_EQ(r.centre(), Point(-1.5, -0.5));
}

TEST(RectangleCornerTests, test_positive_points)
{
    Rectangle r = Rectangle(Point(0, 0), Point(8, 8));
    EXPECT_EQ(r.posXPosYCorner(), Point(8, 8));
    EXPECT_EQ(r.posXNegYCorner(), Point(8, 0));
    EXPECT_EQ(r.negXNegYCorner(), Point(0, 0));
    EXPECT_EQ(r.negXPosYCorner(), Point(0, 8));
}

TEST(RectangleCornerTests, test_negative_points)
{
    Rectangle r = Rectangle(Point(-1, -1), Point(-8, -8));
    EXPECT_EQ(r.posXPosYCorner(), Point(-1, -1));
    EXPECT_EQ(r.posXNegYCorner(), Point(-1, -8));
    EXPECT_EQ(r.negXNegYCorner(), Point(-8, -8));
    EXPECT_EQ(r.negXPosYCorner(), Point(-8, -1));
}

TEST(RectangleCornerTests, test_positive_and_negative_points)
{
    Rectangle r = Rectangle(Point(-4, -3), Point(1, 2));
    EXPECT_EQ(r.posXPosYCorner(), Point(1, 2));
    EXPECT_EQ(r.posXNegYCorner(), Point(1, -3));
    EXPECT_EQ(r.negXNegYCorner(), Point(-4, -3));
    EXPECT_EQ(r.negXPosYCorner(), Point(-4, 2));
}

TEST(RectangleCornerTests, test_min_max_values)
{
    Rectangle r = Rectangle(Point(-4, -3), Point(1, 2));
    EXPECT_EQ(r.xMin(), -4);
    EXPECT_EQ(r.xMax(), 1);
    EXPECT_EQ(r.yMin(), -3);
    EXPECT_EQ(r.yMax(), 2);
}

TEST(RectangleEqualsTests, test_rectangles_equal)
{
    Rectangle r = Rectangle(Point(0, 0), Point(5, 5));
    Rectangle p = Rectangle(Point(5, 5), Point(0, 0));
    EXPECT_TRUE(r == p);
}

TEST(RectangleNotEqualsTests, test_rectangles_not_equal_different_diagonal)
{
    Rectangle r = Rectangle(Point(0, 0), Point(6, 5));
    Rectangle p = Rectangle(Point(5, 5), Point(0, 0));
    EXPECT_FALSE(r == p);
}

TEST(RectangleEqualsTests, test_rectangles_not_equal_different_bottom_left_point)
{
    Rectangle r = Rectangle(Point(1, 1), Point(6, 6));
    Rectangle p = Rectangle(Point(0, 0), Point(5, 5));
    EXPECT_FALSE(r == p);
}

TEST(RectangleExpandTest, test_four_points_doubled)
{
    Rectangle rect({-1, 1}, {1, -1});
    Rectangle expected({-2, 2}, {2, -2});
    EXPECT_EQ(rect.expand(1), expected);
}

TEST(RectangleExpandTest, test_four_points_no_scale)
{
    Rectangle poly({-1, 1}, {1, -1});
    Rectangle expected({-1, 1}, {1, -1});
    EXPECT_EQ(poly.expand(0), expected);
}
