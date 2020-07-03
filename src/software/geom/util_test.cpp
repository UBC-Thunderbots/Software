#include "software/geom/util.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/triangle.h"
#include "software/test_util/test_util.h"
#include "software/time/timestamp.h"

TEST(GeomUtilTest, test_closest_lineseg_point)
{
    Point l1(-1, 1);
    Point l2(1, 1);

    EXPECT_TRUE((closestPointOnSeg(Point(0, 2), l1, l2) - Point(0, 1)).length() <
                0.00001);
    EXPECT_TRUE((closestPointOnSeg(Point(-2, 1.5), l1, l2) - Point(-1, 1)).length() <
                0.00001);

    l1 = Point(-2, 1);
    l2 = Point(1, 2);

    EXPECT_TRUE((closestPointOnSeg(Point(1, 0), l1, l2) - Point(0.4, 1.8)).length() <
                0.00001);
    EXPECT_TRUE(
        (closestPointOnSeg(Point(-1.4, 1.2), l1, l2) - Point(-1.4, 1.2)).length() <
        0.00001);
}

TEST(GeomUtilTest, test_offset_to_line)
{
    Point x0(1, -2);
    Point x1(5, -2);
    Point p(2, -3);

    EXPECT_NEAR(1, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 1);

    EXPECT_NEAR(3, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 0);

    EXPECT_NEAR(2, offsetToLine(x0, x1, p), 1e-5);
}

TEST(GeomUtilTest, test_acuteVertexAngle_angle_over_neg_y_axis)
{
    // Two vectors that form an acute angle over the negative y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((-120)));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_angle_over_pos_y_axis)
{
    // Two vectors that form an acute angle over the positive y-axis

    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((70)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((120)));

    EXPECT_DOUBLE_EQ(50, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_180_degrees)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-90)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((90)));

    EXPECT_DOUBLE_EQ(180, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertexAngle_large_angle_over_neg_x_axis)
{
    Vector v1 = Vector::createFromAngle(Angle::fromDegrees((-95)));
    Vector v2 = Vector::createFromAngle(Angle::fromDegrees((99)));

    EXPECT_DOUBLE_EQ(166, acuteVertexAngle(v1, v2).toDegrees());
}

TEST(GeomUtilTest, test_acuteVertex_angle_between_points)
{
    Point p1(2, 0.5);
    Point p2(1, -0.5);
    Point p3(1, 0.5);
    EXPECT_DOUBLE_EQ(45, acuteVertexAngle(p1, p2, p3).toDegrees());
}
