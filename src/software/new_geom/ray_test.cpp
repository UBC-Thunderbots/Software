#include "software/new_geom/ray.h"

#include <gtest/gtest.h>

TEST(RayTest, default_constructor)
{
    Ray r = Ray();
    EXPECT_EQ(Point(), r.getStart());
    EXPECT_EQ(Vector(), r.getDirection());
}

TEST(RayTest, start_direction_constructor)
{
    Ray r = Ray(Point(1, 2), Vector(2, 3));
    EXPECT_EQ(Point(1, 2), r.getStart());
    EXPECT_EQ(Vector(2, 3), r.getDirection());
}

TEST(RayTest, setters)
{
    Ray r = Ray(Point(3, 2), Vector(0, 1));
    EXPECT_EQ(Point(3, 2), r.getStart());
    EXPECT_EQ(Vector(0, 1), r.getDirection());

    r.setStart(Point(-1.5, 2.3));
    r.setDirection(Vector(4.2, -1.2));
    EXPECT_EQ(Point(-1.5, 2.3), r.getStart());
    EXPECT_EQ(Vector(4.2, -1.2), r.getDirection());
}

TEST(RayTest, rotate)
{
    Ray r = Ray(Point(1, 1), Vector(2, 0));
    r.rotate(Angle::quarter());
    EXPECT_EQ(Point(1, 1), r.getStart());
    EXPECT_EQ(Vector(0, 2), r.getDirection());

    r.rotate(Angle::threeQuarter());
    EXPECT_EQ(Point(1, 1), r.getStart());
    EXPECT_EQ(Vector(2, 0), r.getDirection());
}
