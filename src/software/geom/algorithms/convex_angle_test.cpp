#include "software/geom/algorithms/convex_angle.h"

#include <gtest/gtest.h>

TEST(GeomUtilTest, test_convexAngle_angle_over_neg_y_axis)
{
    // Two vectors that form an convex angle over the negative y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((-120)));

    EXPECT_DOUBLE_EQ(50, convexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_convexAngle_angle_over_pos_y_axis)
{
    // Two vectors that form an convex angle over the positive y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((120)));

    EXPECT_DOUBLE_EQ(50, convexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_convexAngle_180_degrees)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-90)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((90)));

    EXPECT_DOUBLE_EQ(180, convexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_convexAngle_large_angle_over_neg_x_axis)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-95)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((99)));

    EXPECT_DOUBLE_EQ(166, convexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_convexVertex_angle_between_points)
{
    Point p1(2, 0.5);
    Point p2(1, -0.5);
    Point p3(1, 0.5);
    EXPECT_DOUBLE_EQ(45, convexAngle(p1, p2, p3).toDegrees());
}
