#include "software/new_geom/point.h"

#include <gtest/gtest.h>

TEST(CreatePointTests, point_default_constructor_test)
{
    Point p = Point();
    EXPECT_EQ(0, p.x());
    EXPECT_EQ(0, p.y());
}

TEST(CreatePointTests, point_specific_constructor_test)
{
    Point p = Point(1, 2);
    EXPECT_EQ(1, p.x());
    EXPECT_EQ(2, p.y());
}

TEST(CreatePointTests, point_copy_constructor_test)
{
    Point p = Point(3, 4);
    Point q = Point(p);
    EXPECT_EQ(3, q.x());
    EXPECT_EQ(4, q.y());
}

TEST(CreatePointTests, point_from_vector_test)
{
    Vector v = Vector(2, 1);
    Point p  = Point(v);
    EXPECT_EQ(2, p.x());
    EXPECT_EQ(1, p.y());
}

TEST(SetPointTests, point_set_x_and_y_test)
{
    Point p = Point();
    p.setX(6);
    p.setY(5);
    EXPECT_EQ(6, p.x());
    EXPECT_EQ(5, p.y());

    p.set(2, 4);
    EXPECT_EQ(2, p.x());
    EXPECT_EQ(4, p.y());
}

TEST(PointLogicTests, point_dist_from_origin_test)
{
    Point p = Point(3, 4);
    EXPECT_EQ(5, p.distanceFromOrigin());
}

TEST(PointLogicTests, point_dist_from_other_point_test)
{
    Point p = Point(3, 4);
    Point q = Point(-3, -4);
    EXPECT_EQ(10, p.distance(q));
}

TEST(PointLogicTests, rotate_point_test)
{
    Point p = Point(3, 4);
    p       = p.rotate(Angle::quarter());
    EXPECT_DOUBLE_EQ(-4, p.x());
    EXPECT_DOUBLE_EQ(3, p.y());
    p = p.rotate(Angle::half());
    EXPECT_DOUBLE_EQ(4, p.x());
    EXPECT_DOUBLE_EQ(-3, p.y());
    p = p.rotate(Angle::quarter());
    EXPECT_DOUBLE_EQ(3, p.x());
    EXPECT_DOUBLE_EQ(4, p.y());
}

TEST(PointOperatorTests, point_assignment_test)
{
    Point p = Point(2, 3);
    Point q = Point();
    q       = p;
    EXPECT_EQ(2, q.x());
    EXPECT_EQ(3, q.y());
}

TEST(PointOperatorTests, point_vector_sum_test)
{
    Point p  = Point(1, 1);
    Vector v = Vector(-2, -2);
    Point q  = p + v;
    EXPECT_EQ(-1, q.x());
    EXPECT_EQ(-1, q.y());
    q = v + q;
    EXPECT_EQ(-3, q.x());
    EXPECT_EQ(-3, q.y());
    q = q - v;
    EXPECT_EQ(-1, q.x());
    EXPECT_EQ(-1, q.y());
}

TEST(PointOperatorTests, point_vector_sum_set_test)
{
    Point p  = Point(-1, 1);
    Vector v = Vector(-2, -2);
    p += v;
    EXPECT_EQ(-3, p.x());
    EXPECT_EQ(-1, p.y());
    p -= v;
    EXPECT_EQ(-1, p.x());
    EXPECT_EQ(1, p.y());
}

TEST(PointOperatorTests, negate_point_test)
{
    Point p = Point(3, -5);
    Point q = -p;
    EXPECT_EQ(-3, q.x());
    EXPECT_EQ(5, q.y());
}

TEST(PointOperatorTests, point_equality_inequality_test)
{
    Point p = Point();
    Point q = Point(0, 0);
    EXPECT_EQ(p, q);

    q.setX(3);

    EXPECT_FALSE(p == q);
    EXPECT_TRUE(p != q);
}

TEST(CollinearPointsTest, test_collinear_points)
{
    Point p(0, 0);
    Point q(1, 1);
    Point r(5, 5);

    EXPECT_TRUE(Point::collinear(p, q, r));
}

TEST(CollinearPointsTest, test_not_collinear_points)
{
    Point p(0, 0);
    Point q(-2, 5);
    Point r(3, -9);

    EXPECT_FALSE(Point::collinear(p, q, r));
}

TEST(CollinearPointsTest, test_points_collinear_two_identical)
{
    EXPECT_TRUE(Point::collinear(Point(), Point(5, 1), Point(5, 1)));
}

TEST(CollinearPointsTest, test_points_collinear_all_identical)
{
    EXPECT_TRUE(Point::collinear(Point(-4, 3), Point(-4, 3), Point(-4, 3)));
}

TEST(CollinearPointsTest, small_double_precision_error_collinear)
{
    // Make sure small double precision error does not affect collinear
    Point p(7.0 + 1.0 / 3.0, 2);
    Point q(5, -5);
    Point r(8, 4);
    EXPECT_TRUE(Point::collinear(p, q, r));
}

TEST(CollinearPointsTest, vertically_collinear_points)
{
    Point p(202, 15);
    Point q(202, -15);
    Point r(202.00000000000003, -0.5);
    EXPECT_TRUE(Point::collinear(p, q, r));
}

TEST(PointsIntersectionTest, test_intersection)
{
    auto point_of_intersection =
        Point::intersection(Point(0, -3), Point(3, 6), Point(0, 4), Point(1, 8));

    EXPECT_EQ(point_of_intersection.value(), Point(-7, -24));
}

TEST(PointsIntersectionTest, test_overlapping_not_intersecting)
{
    auto point_of_intersection =
        Point::intersection(Point(0, -3), Point(3, 6), Point(1, 0), Point(2, 3));

    EXPECT_FALSE(point_of_intersection.has_value());
}

TEST(PointsIntersectionTest, test_other_intersection)
{
    auto point_of_intersection =
        Point::intersection(Point(-1, -1), Point(5, -1), Point(0, 2), Point(10, 8));

    EXPECT_EQ(point_of_intersection.value(), Point(-5, -1));
}
