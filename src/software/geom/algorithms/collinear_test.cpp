#include "software/geom/algorithms/collinear.h"

#include <gtest/gtest.h>

TEST(CollinearPointsTest, test_collinear_points)
{
    Point p(0, 0);
    Point q(1, 1);
    Point r(5, 5);

    EXPECT_TRUE(collinear(p, q, r));
}

TEST(CollinearPointsTest, test_not_collinear_points)
{
    Point p(0, 0);
    Point q(-2, 5);
    Point r(3, -9);

    EXPECT_FALSE(collinear(p, q, r));
}

TEST(CollinearPointsTest, test_points_collinear_two_identical)
{
    EXPECT_TRUE(collinear(Point(), Point(5, 1), Point(5, 1)));
}

TEST(CollinearPointsTest, test_points_collinear_all_identical)
{
    EXPECT_TRUE(collinear(Point(-4, 3), Point(-4, 3), Point(-4, 3)));
}

TEST(CollinearPointsTest, small_double_precision_error_collinear)
{
    // Make sure small double precision error does not affect collinear
    Point p(7.0 + 1.0 / 3.0, 2);
    Point q(5, -5);
    Point r(8, 4);
    EXPECT_TRUE(collinear(p, q, r));
}

TEST(CollinearPointsTest, vertically_collinear_points)
{
    Point p(202, 15);
    Point q(202, -15);
    Point r(202.00000000000003, -0.5);
    EXPECT_TRUE(collinear(p, q, r));
}

TEST(CollinearPointsTest, test_collinear_segments)
{
    for (unsigned int i = 0; i < 10; ++i)
    {
        Vector v = Vector::createFromAngle(
            Angle::fromDegrees((std::rand() % 360)));  // should be random number here
        Point pointA((std::rand() % 100) / 100.0, (std::rand() % 100) / 100.0);
        Point pointB = pointA + v * (std::rand() % 100) / 100.0;
        Point pointC = pointA - v * (std::rand() % 100) / 100.0;
        bool val     = collinear(pointA, pointB, pointC);
        EXPECT_TRUE(val);
    }
}

TEST(CollinearPointsTest, test_collinear_near_floating_point_error)
{
    Point a1(-3.3649999999999998, 0.58600823341148167);
    Point b1(-3.6996445312500001, 0.36075723266601561);
    Point c1(-2.8700676805557683, 0.91914979063239866);
    EXPECT_TRUE(collinear(a1, b1, c1));

    Point a2(-3.7905116725681518, -1.135);
    Point b2(-4.0008916015625005, -0.80003082275390625);
    Point c2(-3.4690320039460172, -1.6468633731576046);
    EXPECT_TRUE(collinear(a2, b2, c2));
}
