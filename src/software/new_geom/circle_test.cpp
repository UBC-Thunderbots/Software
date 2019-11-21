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

TEST(CircleOperatorTests, circle_set_origin)
{
    Circle c = Circle();
    c.setOrigin(Point(1, 2));
    EXPECT_EQ(Point(1, 2), c.getOrigin());
}

TEST(CircleOperatorTests, circle_set_valid_radius)
{
    Circle c = Circle();
    c.setRadius(3);
    EXPECT_EQ(3, c.getRadius());
}

TEST(CircleOperatorTests, circle_set_invalid_radius)
{
    Circle c = Circle();
    EXPECT_THROW(c.setRadius(-1), std::invalid_argument);
}

TEST(CircleOperatorTests, circle_equals_test)
{
    Circle c = Circle(Point(), 5);
    Circle d;
    d.setRadius(5);
    EXPECT_TRUE(c == d);
}

TEST(CircleOperatorTests, circle_not_equals_through_radius_test)
{
    Circle c = Circle(Point(1, 2), 5);
    Circle d;
    d.setRadius(5);
    EXPECT_TRUE(c != d);
}

TEST(CircleOperatorTests, circle_not_equals_through_origin_test)
{
    Circle c = Circle(Point(1, 2), 5);
    Circle d = Circle(Point(1, 2), 3);
    EXPECT_TRUE(c != d);
}

TEST(CircleAreaTest, circle_area_tests)
{
    Circle c = Circle(Point(), 8);
    EXPECT_LT(201.062 - c.area(), 1e-4);

    Circle zero = Circle();
    EXPECT_EQ(0, zero.area());
}

TEST(CircleContainsPointTests, zero_circle_contans_point_test)
{
    Circle zero = Circle();
    Point p     = Point(1, 1);
    EXPECT_FALSE(zero.contains(p));
    EXPECT_TRUE(zero.contains(Point()));
}

TEST(CircleContainsPointTests, circle_at_origin_contains_point_test)
{
    Circle c = Circle(Point(), 5);
    EXPECT_FALSE(c.contains(Point(-6, -6)));
    EXPECT_TRUE(c.contains(Point(-2, 3)));
}

TEST(CircleContainsPointTests, circle_not_at_origin_contains_point_test)
{
    Circle d = Circle(Point(-5, -5), 5);
    EXPECT_FALSE(d.contains(Point()));
    EXPECT_TRUE(d.contains(Point(-10, -5)));
}
