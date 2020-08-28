#include "software/geom/polynomial2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(Polynomial2dTest, default_constructor)
{
    const Polynomial2d p;

    EXPECT_EQ(Point(0, 0), p.getValueAt(0));
    EXPECT_EQ(Point(0, 0), p.getValueAt(10));
    EXPECT_EQ(Point(0, 0), p.getValueAt(-10));
    EXPECT_EQ(Point(0, 0), p.getValueAt(10000000000));
    EXPECT_EQ(Point(0, 0), p.getValueAt(-10000000000));

    Polynomial1d p_x = p.getPolyX();
    Polynomial1d p_y = p.getPolyY();

    for (int i = -10; i < 10; i++)
    {
        EXPECT_DOUBLE_EQ(p_x.getCoeff(i), 0);
        EXPECT_DOUBLE_EQ(p_x.valueAt(i), 0);
        EXPECT_DOUBLE_EQ(p_y.getCoeff(i), 0);
        EXPECT_DOUBLE_EQ(p_y.valueAt(i), 0);
    }
    EXPECT_DOUBLE_EQ(p_x.getOrder(), 0);
    EXPECT_DOUBLE_EQ(p_y.getOrder(), 0);
}

TEST(Polynomial2dTest, constructor_from_two_1d_polynomials_and_value_at)
{
    const Polynomial1d p_x({0, 2, 5, 7});
    const Polynomial1d p_y({0, 3, 1, 8});
    const Polynomial2d p(p_x, p_y);

    const double t_val = 0.33;
    const Point expected_point(p_x.valueAt(t_val), p_y.valueAt(t_val));
    EXPECT_EQ(expected_point, p.getValueAt(t_val));
}

TEST(Polynomial2dTest, constructor_from_list_of_points_less_then_two_points)
{
    const std::vector<Point> points = {Point(-1, 1)};

    EXPECT_THROW(Polynomial2d p(points), std::invalid_argument);
}

TEST(Polynomial2dTest, constructor_from_list_of_points_valid)
{
    const std::vector<Point> points = {Point(-1, 1), Point(0, 0), Point(3.3, 7.8)};

    const Polynomial2d p(points);

    // We should only need a second order polynomial in x and y to interpolate three
    // points
    EXPECT_EQ(2, p.getPolyX().getOrder());
    EXPECT_EQ(2, p.getPolyY().getOrder());

    // Check that we interpolate the start and end points at the expected t-values
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(-1, 1), p.getValueAt(0), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(3.3, 7.8), p.getValueAt(1), 1e-9));

    // Check that the intermediate point was interpolated correctly
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(0, 0), p.getValueAt(0.5), 1e-9));

    // Check the coefficients are as expected
    // These were calculated using an online math tool
    // (https://www.wolframalpha.com/)
    // NOTE: The tolerances used here are very tight because several many commonly used
    //       methods for solving for these polynomials can have significant numerical
    //       error. Please do not loosen the tolerance unless you really know what you're
    //       doing.
    EXPECT_DOUBLE_EQ(-1, p.getPolyX().getCoeff(0));
    EXPECT_DOUBLE_EQ(-0.3, p.getPolyX().getCoeff(1));
    EXPECT_DOUBLE_EQ(4.6, p.getPolyX().getCoeff(2));
    EXPECT_DOUBLE_EQ(1, p.getPolyY().getCoeff(0));
    EXPECT_DOUBLE_EQ(-10.8, p.getPolyY().getCoeff(1));
    EXPECT_DOUBLE_EQ(17.6, p.getPolyY().getCoeff(2));
}

TEST(Polynomial2dTest, constructor_from_initializer_list_of_points)
{
    // Here we just check that the initializer list constructor is equivalent to the
    // vector constructor
    const std::vector<Point> points = {Point(-1, 1), Point(0, 0), Point(3.3, 7.8)};

    const Polynomial2d initializer_list_poly(
        {Point(-1, 1), Point(0, 0), Point(3.3, 7.8)});
    const Polynomial2d vector_poly(points);

    EXPECT_EQ(vector_poly.getPolyX(), initializer_list_poly.getPolyX());
    EXPECT_EQ(vector_poly.getPolyY(), initializer_list_poly.getPolyY());
}

TEST(Polynomial2dTest, get_poly_x)
{
    const Polynomial1d p_x({0, 2, 5, 7});
    const Polynomial1d p_y({0, 3, 1, 8});
    const Polynomial2d p(p_x, p_y);

    EXPECT_EQ(p_x, p.getPolyX());
}

TEST(Polynomial2dTest, get_poly_y)
{
    const Polynomial1d p_x({0, 2, 5, 7});
    const Polynomial1d p_y({0, 3, 1, 8});
    const Polynomial2d p(p_x, p_y);

    EXPECT_EQ(p_y, p.getPolyY());
}

TEST(Polynomial2dTest, equality_operator_equal)
{
    const Polynomial1d p_x({0, 2, 5, 7});
    const Polynomial1d p_y({0, 3, 1, 8});
    const Polynomial2d p1(p_x, p_y);
    const Polynomial2d p2(p_x, p_y);

    EXPECT_TRUE(operator==(p1, p2));
}

TEST(Polynomial2dTest, equality_operator_x_poly_not_equal)
{
    const Polynomial1d p_x_1({0, 2, 5, 7});
    const Polynomial1d p_x_2({0, 1, 5, 7});
    const Polynomial1d p_y({0, 3, 1, 8});
    const Polynomial2d p1(p_x_1, p_y);
    const Polynomial2d p2(p_x_2, p_y);

    EXPECT_FALSE(operator==(p1, p2));
}

TEST(Polynomial2dTest, equality_operator_y_poly_not_equal)
{
    const Polynomial1d p_x({0, 2, 5, 7});
    const Polynomial1d p_y_1({0, 2, 1, 8});
    const Polynomial1d p_y_2({0, 3, 1, 8});
    const Polynomial2d p1(p_x, p_y_1);
    const Polynomial2d p2(p_x, p_y_2);

    EXPECT_FALSE(operator==(p1, p2));
}

TEST(Polynomial2dTest, addition_operator)
{
    const Polynomial2d p1(Polynomial1d({1, 2, 3}), Polynomial1d({1, 1, 1}));

    const Polynomial2d p2(Polynomial1d({2, 4, 6}), Polynomial1d({1, 2, 4}));

    const Polynomial2d expected(Polynomial1d({3, 6, 9}), Polynomial1d({2, 3, 5}));

    EXPECT_EQ(expected, p1 + p2);
}

TEST(Polynomial2dTest, subtraction_operator)
{
    const Polynomial2d p1(Polynomial1d({5, 6, 7}), Polynomial1d({7, 5, 9}));

    const Polynomial2d p2(Polynomial1d({2, 4, 6}), Polynomial1d({1, 2, 4}));

    const Polynomial2d expected(Polynomial1d({3, 2, 1}), Polynomial1d({6, 3, 5}));

    EXPECT_EQ(expected, p1 - p2);
}

TEST(Polynomial2dTest, addition_assignment_operator)
{
    const Polynomial2d p1(Polynomial1d({1, 2, 3}), Polynomial1d({1, 1, 1}));

    Polynomial2d p2(Polynomial1d({2, 4, 6}), Polynomial1d({1, 2, 4}));

    const Polynomial2d expected(Polynomial1d({3, 6, 9}), Polynomial1d({2, 3, 5}));

    p2 += p1;

    EXPECT_EQ(expected, p2);
}

TEST(Polynomial2dTest, subtraction_assignment_operator)
{
    const Polynomial2d p1(Polynomial1d({1, 2, 3}), Polynomial1d({1, 1, 1}));

    Polynomial2d p2(Polynomial1d({6, 8, 10}), Polynomial1d({5, 2, 3}));

    const Polynomial2d expected(Polynomial1d({5, 6, 7}), Polynomial1d({4, 1, 2}));

    p2 -= p1;

    EXPECT_EQ(expected, p2);
}
