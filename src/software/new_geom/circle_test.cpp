#include "software/new_geom/circle.h"

#include <gtest/gtest.h>

TEST(CreateCircleTests, circle_default_constructor)
{
    Circle c = Circle();
    EXPECT_EQ(0, c.getRadius());
    EXPECT_EQ(Point(), c.getOrigin());
}

TEST(CreateCircleTests, circle_custom_constructor)
{
    Circle c = Circle(Point(1, 2), 5);
    EXPECT_EQ(5, c.getRadius());
    EXPECT_EQ(Point(1, 2), c.getOrigin());

    EXPECT_THROW(Circle(Point(3, 5), -4), std::invalid_argument);
}

TEST(CircleLogicTests, circle_getter_setter_tests)
{
    Circle c = Circle();
    c.setOrigin(Point(2, -3));
    c.setRadius(10);
    EXPECT_EQ(Point(2, -3), c.getOrigin());
    EXPECT_EQ(10, c.getRadius());
    c.setOrigin(Point(-6, 7));
    EXPECT_EQ(Point(-6, 7), c.getOrigin());

    EXPECT_THROW(c.setRadius(-1), std::invalid_argument);
}

TEST(CircleOperatorTests, circle_operator_tests)
{
    Circle c = Circle(Point(1, 2), 3);
    Circle d;
    d.setOrigin(Point(1, 2));
    d.setRadius(3);
    EXPECT_TRUE(c == d);

    d.setRadius(4);
    EXPECT_TRUE(c != d);

    d.setRadius(3);
    d.setOrigin(Point());
    EXPECT_FALSE(c == d);
}

TEST(CircleAreaTest, circle_area_tests)
{
    Circle c = Circle(Point(), 8);
    EXPECT_LT(201.062 - c.area(), 1e-4);

    Circle zero = Circle();
    EXPECT_EQ(0, zero.area());
}

TEST(CircleContainsPointTest, circle_contains_point_tests)
{
    Circle zero = Circle();
    Point p = Point(1, 1);
    EXPECT_FALSE(zero.contains(p));
    EXPECT_TRUE(zero.contains(Point()));

    Circle c = Circle(Point(), 5);
    EXPECT_FALSE(c.contains(Point(-6, -6)));
    EXPECT_TRUE(c.contains(Point(-2, 3)));

    Circle d = Circle(Point(-5, -5), 5);
    EXPECT_FALSE(d.contains(Point()));
    EXPECT_TRUE(d.contains(Point(-10, -5)));
}
