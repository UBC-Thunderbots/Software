#include "software/new_geom/util/circle_tangent.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(CircleTangentTest, test_circle_tangent_points)
{
    Point p(2, 3);
    Circle c(Point(-1, 0), 2);
    auto [p1, p2] = getCircleTangentPoints(p, c);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(p1, Point(-1.58, 1.91), 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(p2, Point(0.91, -0.58), 0.01));
}

TEST(CircleTangentTest, test_circle_tangent_rays)
{
    Point p(2, 3.5);
    Circle c(Point(-1, 0), 2);
    auto [r1, r2] = getCircleTangentRaysWithReferenceOrigin(p, c);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r1.getStart(), p, 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r2.getStart(), p, 0.01));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r1.getDirection(),
                                               (Point(-1.58, 1.91) - p).orientation(),
                                               Angle::fromRadians(0.01)));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(r2.getDirection(),
                                               (Point(0.91, -0.58) - p).orientation(),
                                               Angle::fromRadians(0.01)));
}
