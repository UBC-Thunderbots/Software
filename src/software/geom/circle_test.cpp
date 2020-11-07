#include "software/geom/circle.h"

#include <gtest/gtest.h>

TEST(CreateCircleTests, circle_default_constructor)
{
    Circle c = Circle();
    EXPECT_EQ(0, c.radius());
    EXPECT_EQ(Point(), c.origin());
}

TEST(CreateCircleTests, circle_custom_constructor)
{
    Circle c = Circle(Point(1, 2), 5);
    EXPECT_EQ(5, c.radius());
    EXPECT_EQ(Point(1, 2), c.origin());
}

TEST(CreateCircleTests, circle_invalid_input)
{
    EXPECT_THROW(Circle(Point(1, 2), -1), std::invalid_argument);
}

TEST(CircleOperatorTests, circle_equals_through_origin_test)
{
    Circle c = Circle(Point(1, 2), 5);
    Circle d = Circle(Point(1, 2), 5);
    EXPECT_TRUE(c == d);
}

TEST(CircleOperatorTests, circle_not_equals_through_origin_test)
{
    Circle c = Circle(Point(1, 2), 5);
    Circle d = Circle(Point(1, 2), 3);
    Circle e = Circle(Point(0, 2), 5);
    EXPECT_TRUE(c != d);
    EXPECT_TRUE(c != e);
}

TEST(CircleAreaTest, circle_area_tests)
{
    Circle c = Circle(Point(), 8);
    EXPECT_LT(201.062 - c.area(), 1e-4);

    Circle zero = Circle();
    EXPECT_EQ(0, zero.area());
}
