#include "software/new_geom/ray.h"

#include <gtest/gtest.h>

TEST(RayTest, default_constructor)
{
    Ray r = Ray();
    EXPECT_EQ(Point(), r.getStart());
    EXPECT_EQ(Angle::zero(), r.getDirection());
}

TEST(RayTest, angle_constructor)
{
    Ray r = Ray(Point(1, 2), Angle::half());
    EXPECT_EQ(Point(1, 2), r.getStart());
    EXPECT_EQ(Angle::half(), r.getDirection());
}

TEST(RayTest, vector_constructor)
{
    Ray r = Ray(Point(1, 2), Vector(2, 3));
    EXPECT_EQ(Point(1, 2), r.getStart());
    EXPECT_EQ(Vector(2, 3).orientation(), r.getDirection());
}

TEST(RayTest, vector_constructor_zero_vector)
{
    Ray r = Ray(Point(1, 2), Vector());
    EXPECT_EQ(Point(1, 2), r.getStart());
    EXPECT_EQ(Vector().orientation(), r.getDirection());
}

TEST(RayTest, point_setter)
{
    Ray r = Ray(Point(3, 2), Angle::half());
    EXPECT_EQ(Point(3, 2), r.getStart());

    r.setStart(Point(-1.5, 2.3));
    EXPECT_EQ(Point(-1.5, 2.3), r.getStart());
}

TEST(RayTest, angle_direction_setter)
{
    Ray r = Ray(Point(3, 2), Angle::quarter());
    EXPECT_EQ(Angle::quarter(), r.getDirection());

    r.setDirection(Angle::fromDegrees(60));
    EXPECT_EQ(Angle::fromDegrees(60), r.getDirection());
}

TEST(RayTest, vector_direction_setter)
{
    Ray r = Ray(Point(3, 2), Angle::fromRadians(0.5));
    EXPECT_EQ(Angle::fromRadians(0.5), r.getDirection());

    r.setDirection(Vector(4.2, -1.2));
    EXPECT_EQ(Vector(4.2, -1.2).orientation(), r.getDirection());
}

TEST(RayTest, rotate)
{
    Ray r = Ray(Point(1, 1), Angle::zero());
    r.rotate(Angle::quarter());
    EXPECT_EQ(Point(1, 1), r.getStart());
    EXPECT_EQ(Angle::quarter(), r.getDirection());

    r.rotate(Angle::threeQuarter());
    EXPECT_EQ(Point(1, 1), r.getStart());
    EXPECT_EQ(Angle::zero(), r.getDirection());
}

TEST(RayTest, to_unit_vector)
{
    Angle a = Angle::fromDegrees(30);
    Ray r   = Ray(Point(1, 1), a);
    EXPECT_EQ(r.toUnitVector(), Vector(sqrt(3), 1).normalize());
}

TEST(RayContainsPointTest, test_ray_contains_point_no_x_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, 0.5);

    EXPECT_TRUE(ray.contains(point));
}

TEST(RayContainsPointTest, test_ray_doesnt_contain_point_behind_start)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, -0.5);

    EXPECT_FALSE(ray.contains(point));
}

TEST(RayContainsPointTest, test_ray_doesnt_contain_point)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(-5, 10);

    EXPECT_FALSE(ray.contains(point));
}

TEST(RayContainsPointTest, test_ray_contains_point_no_y_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(1, 0));
    Point point = Point(0.5, 0);

    EXPECT_TRUE(ray.contains(point));
}

TEST(RayContainsPointTest, test_diagonal_ray_contains_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(1, 1);

    EXPECT_TRUE(ray.contains(point));
}

TEST(RayContainsPointTest, test_ray_contains_distant_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(-20, -20);

    EXPECT_TRUE(ray.contains(point));
}

TEST(RayContainsPointTest, test_ray_contains_ray_start)
{
    Ray ray = Ray(Point(2, 2), Vector(-1, -1));

    EXPECT_TRUE(ray.contains(ray.getStart()));
}
