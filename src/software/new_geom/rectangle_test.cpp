#include "software/new_geom/rectangle.h"

#include <gtest/gtest.h>

TEST(RectangleConstructorTests, test_bottom_left_top_right)
{
    Rectangle r = Rectangle(Point(0, 0), Point(4, 5));
    EXPECT_EQ(r.xLength(), 4);
    EXPECT_EQ(r.yLength(), 5);
    for (const Point &p : r.getPoints()) {
        std::cout << p.x() << " and " << p.y() << std::endl;
    }
}

TEST(RectangleConstructorTests, test_top_left_bottom_right)
{
    Rectangle r = Rectangle(Point(-3, 3), Point(4, -5));
    EXPECT_EQ(r.xLength(), 7);
    EXPECT_EQ(r.yLength(), 8);
    for (const Point &p : r.getPoints()) {
        std::cout << p.x() << " and " << p.y() << std::endl;
    }
}

TEST(RectangleConstructorTests, test_top_right_bottom_left)
{
    Rectangle r = Rectangle(Point(2, 7), Point(0, 0));
    EXPECT_EQ(r.xLength(), 2);
    EXPECT_EQ(r.yLength(), 7);
    for (const Point &p : r.getPoints()) {
        std::cout << p.x() << " and " << p.y() << std::endl;
    }
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

TEST(RectangleSubscriptOperatorTests, test_subscript_operator)
{
    Rectangle r = Rectangle(Point(-4, -3), Point(1, 2));
    EXPECT_EQ(r[0], Point(-4, -3));
    EXPECT_EQ(r[1], Point(-4, 2));
    EXPECT_EQ(r[2], Point(1, 2));
    EXPECT_EQ(r[3], Point(1, -3));
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
