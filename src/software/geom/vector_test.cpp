#include "software/geom/vector.h"

#include <gtest/gtest.h>

#include <chrono>

#include "software/geom/point.h"

TEST(CreateVectorTests, vector_default_constructor_test)
{
    Vector v = Vector();
    EXPECT_EQ(0, v.x());
    EXPECT_EQ(0, v.y());
}

TEST(CreateVectorTests, vector_from_angle)
{
    Vector v = Vector::createFromAngle(Angle::fromDegrees(30));
    EXPECT_DOUBLE_EQ(sqrt(3) / 2, v.x());
    EXPECT_DOUBLE_EQ(0.5, v.y());
}

TEST(CreateVectorTests, vector_specific_constructor_test)
{
    Vector v = Vector(-3, 4);
    EXPECT_EQ(-3, v.x());
    EXPECT_EQ(4, v.y());
}

TEST(CreateVectorTests, vector_copy_constructor_test)
{
    Vector v = Vector(5, -2);
    Vector u = Vector(v);
    EXPECT_EQ(5, u.x());
    EXPECT_EQ(-2, u.y());
}

TEST(CreateVectorTests, vector_from_point_test)
{
    Point p  = Point(-3, 7);
    Vector v = p.toVector();
    EXPECT_EQ(-3, v.x());
    EXPECT_EQ(7, v.y());
}

TEST(SetVectorTests, vector_set_x_and_y_test)
{
    Vector v = Vector();
    v.setX(6);
    v.setY(5);
    EXPECT_EQ(6, v.x());
    EXPECT_EQ(5, v.y());

    v.set(2, 4);
    EXPECT_EQ(2, v.x());
    EXPECT_EQ(4, v.y());
}

TEST(VectorLogicTests, vector_len_and_lensq_test)
{
    Vector v = Vector(3, 4);
    EXPECT_EQ(25, v.lengthSquared());
    EXPECT_EQ(5, v.length());
}

TEST(VectorLogicTests, norm_vector_from_vector_test)
{
    Vector u = Vector(3, 4);
    Vector v = u.normalize();
    EXPECT_EQ(0.6, v.x());
    EXPECT_EQ(0.8, v.y());
}

TEST(VectorLogicTests, norm_vector_from_vector_with_length_test)
{
    Vector u = Vector(3, 4);
    Vector v = u.normalize(2);
    EXPECT_EQ(1.2, v.x());
    EXPECT_EQ(1.6, v.y());
}

TEST(VectorLogicTests, norm_near_zero_vector_test)
{
    Vector u = Vector(FIXED_EPSILON * 0.99, FIXED_EPSILON * 0.99);
    Vector v = u.normalize();
    EXPECT_EQ(Vector(), v);
}

TEST(VectorLogicTests, vector_perp_test)
{
    Vector u = Vector(3, 4);
    Vector v = u.perpendicular();
    EXPECT_EQ(-4, v.x());
    EXPECT_EQ(3, v.y());
}

TEST(VectorLogicTests, rotate_vector_test)
{
    Vector v = Vector(3, 4);
    v        = v.rotate(Angle::quarter());
    EXPECT_DOUBLE_EQ(-4, v.x());
    EXPECT_DOUBLE_EQ(3, v.y());
    v = v.rotate(Angle::half());
    EXPECT_DOUBLE_EQ(4, v.x());
    EXPECT_DOUBLE_EQ(-3, v.y());
    v = v.rotate(Angle::quarter());
    EXPECT_DOUBLE_EQ(3, v.x());
    EXPECT_DOUBLE_EQ(4, v.y());
}

TEST(VectorLogicTests, vector_proj_test)
{
    Vector u    = Vector(1, 2);
    Vector v    = Vector(3, 4);
    Vector proj = u.project(v);
    EXPECT_EQ(1.32, proj.x());
    EXPECT_EQ(1.76, proj.y());
}

TEST(VectorLogicTests, project_vector_onto_vector_in_opposite_direction)
{
    Vector u    = Vector(1, 2);
    Vector v    = Vector(-3, 0);
    Vector proj = u.project(v);
    EXPECT_DOUBLE_EQ(1.0, proj.x());
    EXPECT_DOUBLE_EQ(0.0, proj.y());
}

TEST(VectorLogicTests, vector_dot_test)
{
    Vector u = Vector(-4, -9);
    Vector v = Vector(-1, 2);
    EXPECT_DOUBLE_EQ(-14, u.dot(v));
}

TEST(VectorLogicTests, vector_cross_product_test)
{
    Vector u = Vector(-4, -9);
    Vector v = Vector(-1, 2);
    EXPECT_DOUBLE_EQ(-17, u.cross(v));
}

TEST(VectorLogicTests, vector_orientation_test)
{
    Vector u = Vector(sqrt(2) / 2, sqrt(2) / 2);
    EXPECT_EQ(Angle::fromDegrees(45), u.orientation());
    Vector v = Vector(0, -1);
    EXPECT_EQ(Angle::fromDegrees(-90), v.orientation());
}

TEST(VectorOperatorTests, vector_assignment_test)
{
    Vector u = Vector(2, 3);
    Vector v = Vector();
    v        = u;
    EXPECT_EQ(2, v.x());
    EXPECT_EQ(3, v.y());
}

TEST(VectorOperatorTests, vector_sum_test)
{
    Vector u = Vector(2, 3);
    Vector v = Vector(-3, 5);
    Vector w = u + v;
    EXPECT_EQ(-1, w.x());
    EXPECT_EQ(8, w.y());
}

TEST(VectorOperatorTests, vector_sum_set_test)
{
    Vector u = Vector(2, 3);
    Vector v = Vector(-3, 5);
    u += v;
    EXPECT_EQ(-1, u.x());
    EXPECT_EQ(8, u.y());
}

TEST(VectorOperatorTests, negate_vector_test)
{
    Vector u = Vector(3, -5);
    Vector v = -u;
    EXPECT_EQ(-3, v.x());
    EXPECT_EQ(5, v.y());
}

TEST(VectorOperatorTests, vector_subtraction_test)
{
    Vector u = Vector(2, 3);
    Vector v = Vector(-3, 5);
    Vector w = u - v;
    EXPECT_EQ(5, w.x());
    EXPECT_EQ(-2, w.y());
}

TEST(VectorOperatorTests, vector_subtraction_set_test)
{
    Vector u = Vector(2, 3);
    Vector v = Vector(-3, 5);
    u -= v;
    EXPECT_EQ(5, u.x());
    EXPECT_EQ(-2, u.y());
}

TEST(VectorOperatorTests, scale_vector_test)
{
    Vector u = Vector(2, -3);
    Vector w = u * 3;
    EXPECT_EQ(6, w.x());
    EXPECT_EQ(-9, w.y());
}

TEST(VectorOperatorTests, scale_set_vector_test)
{
    Vector u = Vector(2, -3);
    u *= 3;
    EXPECT_EQ(6, u.x());
    EXPECT_EQ(-9, u.y());
}

TEST(VectorOperatorTests, divide_vector_test)
{
    Vector u = Vector(12, -3);
    Vector w = u / 3;
    EXPECT_EQ(4, w.x());
    EXPECT_EQ(-1, w.y());
}

TEST(VectorOperatorTests, divide_set_vector_test)
{
    Vector u = Vector(12, -3);
    u /= 3;
    EXPECT_EQ(4, u.x());
    EXPECT_EQ(-1, u.y());
}

TEST(VectorOperatorTests, vector_equality_inequality_test)
{
    Vector u = Vector();
    Vector v = Vector(0, 0);
    EXPECT_EQ(u, v);

    v.setX(3);

    EXPECT_FALSE(u == v);
    EXPECT_TRUE(u != v);
}

TEST(VectorLogicTests, vector_clockwise_of_acute_angle_test)
{
    Vector u = Vector::createFromAngle(Angle::fromDegrees(30));
    Vector v = Vector::createFromAngle(Angle::fromDegrees(60));

    EXPECT_TRUE(u.isClockwiseOf(v));
}

TEST(VectorLogicTests, vector_counterclockwise_of_acute_angle_test)
{
    Vector u = Vector::createFromAngle(Angle::zero());
    Vector v = Vector::createFromAngle(Angle::fromDegrees(45));

    EXPECT_TRUE(v.isCounterClockwiseOf(u));
}

TEST(VectorLogicTests, vector_clockwise_of_obtuse_angle_expect_failure_test)
{
    Vector u = Vector::createFromAngle(Angle::fromDegrees(10));
    Vector v = Vector::createFromAngle(Angle::fromDegrees(110));

    EXPECT_FALSE(v.isClockwiseOf(u));
}

TEST(VectorLogicTests, vector_clockwise_of_acute_angle_expect_failure_test)
{
    Vector u = Vector::createFromAngle(Angle::fromDegrees(-45));
    Vector v = Vector::createFromAngle(Angle::fromDegrees(-60));

    EXPECT_FALSE(u.isClockwiseOf(v));
}

TEST(VectorLogicTests, vector_counterclockwise_of_acute_angle_expect_failure_test)
{
    Vector u = Vector::createFromAngle(Angle::threeQuarter());
    Vector v = Vector::createFromAngle(Angle::full());

    EXPECT_FALSE(u.isCounterClockwiseOf(v));
}

TEST(VectorLogicTests, vector_counterclockwiseof_obtuse_angle_expect_failure_test)
{
    Vector u = Vector::createFromAngle(Angle::zero());
    Vector v = Vector::createFromAngle(Angle::fromDegrees(-91));

    EXPECT_FALSE(v.isCounterClockwiseOf(u));
}

TEST(VectorLogicTests, vector_clockwise_of_right_angle_expect_success_test)
{
    Vector u = Vector::createFromAngle(Angle::zero());
    Vector v = Vector::createFromAngle(Angle::fromDegrees(90));

    EXPECT_TRUE(u.isClockwiseOf(v));
}

TEST(VectorLogicTests, vector_counterclockwise_of_right_angle_expect_failure_test)
{
    Vector u = Vector::createFromAngle(Angle::fromDegrees(-90));
    Vector v = Vector::createFromAngle(Angle::fromDegrees(-180));

    EXPECT_FALSE(v.isCounterClockwiseOf(u));
}

TEST(VectorLogicTests, vector_counterclockwise_of_in_quadrant_ii_test)
{
    Vector u = Vector(1, -1);
    Vector v = Vector(1, -0.1);

    EXPECT_TRUE(v.isCounterClockwiseOf(u));
}
