#include "software/geom/ray.h"

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
