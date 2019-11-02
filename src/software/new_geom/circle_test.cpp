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
}

TEST(CircleSetterTests, circle_logic_tests)
{
    Circle c = Circle();
    c.setOrigin(Point(2,- 3));
    c.setRadius(10);
    EXPECT_EQ(Point(2, -3), c.getOrigin());
    EXPECT_EQ(10, c.getRadius());
    c.setOrigin(Point(-6, 7));
    EXPECT_EQ(Point(-6, 7), c.getOrigin());
    c.setRadius(8);
    EXPECT_EQ(8, c.getRadius());
    EXPECT_EQ(201.062, c.area());

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
