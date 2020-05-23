#include "software/new_geom/rectangle.h"

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

TEST(RectangleContainsPointTests, test_point_in_different_quadrant)
{
    EXPECT_FALSE(Rectangle(Point(0, 0), Point(-2, -2)).contains(Point(1, 1)));
    EXPECT_TRUE(Rectangle(Point(0, 0), Point(-2, -2)).contains(Point(-1, -1)));
}

TEST(RectangleContainsPointTests, test_point_in_same_quadrant)
{
    EXPECT_TRUE(Rectangle(Point(0, 0), Point(3, 3)).contains(Point(1, 2)));
    EXPECT_TRUE(Rectangle(Point(0, 0), Point(-4, 4)).contains(Point(-2, 3)));
}

TEST(RectangleContainsPointTests, test_point_on_rectangle_corner)
{
    EXPECT_TRUE(Rectangle(Point(0, 0), Point(2, 2)).contains(Point(0, 0)));
}

TEST(RectangleContainsPointTests, test_point_on_rectangle_edge)
{
    EXPECT_TRUE(Rectangle(Point(0, 0), Point(3, 3)).contains(Point(3, 3)));
}

TEST(RectangleContainsPointTests, test_point_off_left_of_rectangle)
{
    EXPECT_FALSE(Rectangle(Point(0, 0), Point(-4, 4)).contains(Point(-7, 2)));
}

TEST(RectangleContainsPointTests, test_point_off_right_of_rectangle)
{
    EXPECT_FALSE(Rectangle(Point(0, 0), Point(-4, 4)).contains(Point(1, 0)));
}

TEST(RectangleContainsPointTests, test_point_off_below_rectangle)
{
    EXPECT_FALSE(Rectangle(Point(0, 0), Point(-4, 4)).contains(Point(-2, -1)));
}

TEST(RectangleContainsPointTests, test_point_off_above_rectangle)
{
    EXPECT_FALSE(Rectangle(Point(0, 0), Point(-4, 4)).contains(Point(-2, 5)));
}

TEST(RectangleContainsPointTests, test_point_centre_of_rectangle)
{
    EXPECT_TRUE(Rectangle(Point(1, 1), Point(-1, -1)).contains(Point(0.5, 0.5)));
}

TEST(RectangleFurthestCornerTests, test_rect_furthest_corner_top_right)
{
    EXPECT_EQ(Rectangle(Point(0, 0), Point(8, 4)).furthestCorner(Point(0.5, 0.5)),
              Point(8, 4));
}

TEST(RectangleFurthestCornerTests, test_rect_furthest_corner_top_left)
{
    EXPECT_EQ(Rectangle(Point(0, 0), Point(8, 4)).furthestCorner(Point(7, 0.5)),
              Point(0, 4));
}

TEST(RectangleFurthestCornerTests, test_rect_furthest_corner_bottom_right)
{
    EXPECT_EQ(Rectangle(Point(0, 0), Point(8, 4)).furthestCorner(Point(0.5, 3)),
              Point(8, 0));
}

TEST(RectangleFurthestCornerTests, test_rect_furthest_corner_bottom_left)
{
    EXPECT_EQ(Rectangle(Point(0, 0), Point(8, 4)).furthestCorner(Point(7, 3)),
              Point(0, 0));
}

TEST(RectangleExpandTests, test_expand_positive)
{
    Rectangle r = Rectangle(Point(2, -2), Point(-3, 5));
    EXPECT_EQ(r.xLength(), 5);
    EXPECT_EQ(r.yLength(), 7);
    r.inflate(4);
    EXPECT_EQ(r.xLength(), 13);
    EXPECT_EQ(r.yLength(), 15);
}

TEST(RectangleExpandTests, test_expand_negative)
{
    Rectangle r = Rectangle(Point(-3, 3), Point(4, -5));
    EXPECT_EQ(r.xLength(), 7);
    EXPECT_EQ(r.yLength(), 8);
    r.inflate(-2);
    EXPECT_EQ(r.xLength(), 3);
    EXPECT_EQ(r.yLength(), 4);
}

TEST(RectangleExpandTests, test_invalid_expand)
{
    Rectangle r = Rectangle(Point(-3, 3), Point(4, -5));
    EXPECT_EQ(r.xLength(), 7);
    EXPECT_EQ(r.yLength(), 8);
    EXPECT_FALSE(r.inflate(-5));
    EXPECT_EQ(r.xLength(), 7);
    EXPECT_EQ(r.yLength(), 8);
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

TEST(RectangleExpandTest, test_right)
{
    Rectangle rectangle({1, 1}, {5, 3});
    Rectangle expected({1, 1}, {8, 3});
    Vector expansion_vector({3, 0});
    EXPECT_EQ(rectangle.expand(expansion_vector), expected);
}

TEST(RectangleExpandTest, test_up_left)
{
    Rectangle rectangle({1, 1}, {5, 3});
    Rectangle expected({-1, 1}, {5, 6});
    Vector expansion_vector({-2, 3});
    EXPECT_EQ(rectangle.expand(expansion_vector), expected);
}

TEST(RectangleExpandTest, test_down_right)
{
    Rectangle rectangle({1, 1}, {5, 3});
    Rectangle expected({1, 0}, {9, 3});
    Vector expansion_vector({4, -1});
    EXPECT_EQ(rectangle.expand(expansion_vector), expected);
}

TEST(RectangleExpandTest, test_0_vector)
{
    Rectangle rectangle({1, 1}, {5, 3});
    Rectangle expected(rectangle);
    Vector expansion_vector({0, 0});
    EXPECT_EQ(rectangle.expand(expansion_vector), expected);
}
