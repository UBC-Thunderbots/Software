/*
 * Test file for trespass functions
 */

#include "ai/navigator/trespass.h"

#include <gtest/gtest.h>

#include "geom/point.h"
#include "geom/rectangle.h"

TEST(TrespassTest, calcLinearTrespassScore_rectangle_test)
{
    Rectangle rect   = Rectangle(Point(0, 0), 6, 2);
    Point outside    = Point(-1, -2);
    Point corner     = Point(0, 0);
    Point edge_e     = Point(6, 1);
    Point edge_s     = Point(3, 0);
    Point centre     = Point(3, 1);
    Point quarter    = Point(1.5, 0.5);
    Point quarter_y  = Point(3, 0.5);
    Point two_thirds = Point(4, 0.2);

    // Test point outside the rectangle. Expect the function to return 0.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, outside), 0);

    // Test point on the corner of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, corner), 0);

    // Test point halfway along the right edge of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, edge_e), 0);

    // Test point halfway along the bottom edge of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, edge_s), 0);

    // Test point at the centre of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, centre), 1);

    // Test point at a quarter of the width and height of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, quarter), 0.5);

    // Test point at a quarter of the height but at half the width of the rectangle.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, quarter_y), 1);

    // Test point at two-thirds the width of the rectangle and one tenth the height.
    EXPECT_EQ(Navigator::Trespass::calcLinearTrespassScore(rect, two_thirds), 2.0 / 3.0);
}

// Test to see if the 1 is returned when the point exists within the rectangle
TEST(TrespassTest, test_binary_trespass_point_is_trespassing_in_rectangle)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(0, 0)));
    EXPECT_EQ(1,
              Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(0.5, 0.5)));
    EXPECT_EQ(1,
              Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-0.5, -0.5)));
    EXPECT_EQ(1,
              Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(0.5, -0.5)));
    EXPECT_EQ(1,
              Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-0.5, 0.5)));
}

// Test to see if the 1 is returned when the point exists on the boundries of the
// rectangle
TEST(TrespassTest, test_binary_trespass_point_is_on_rectangle_boundry)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-1, -1)));
    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(1, 1)));
    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-1, 1)));
    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(1, -1)));
    EXPECT_EQ(1, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-1, 0.5)));
}

// Test to see if the 0 is returned when the point exists outside of the rectangle
TEST(TrespassTest, test_binary_trespass_point_is_outside_rectangle)
{
    Rectangle rectangle = Rectangle(Point(-1, -1), Point(1, 1));

    EXPECT_EQ(0, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-1, -2)));
    EXPECT_EQ(0, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(2, 1)));
    EXPECT_EQ(0, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-1, 3)));
    EXPECT_EQ(0, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(5, -0.2)));
    EXPECT_EQ(0, Navigator::Trespass::calcBinaryTrespassScore(rectangle, Point(-4, 5)));
}
