#include "software/new_geom/polynomial2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

using namespace ::Test;

TEST(Polynomial2dTest, default_constructor)
{
    const Polynomial2d p;

    EXPECT_EQ(Point(0, 0), p.valueAt(0));
    EXPECT_EQ(Point(0, 0), p.valueAt(10));
    EXPECT_EQ(Point(0, 0), p.valueAt(-10));
    EXPECT_EQ(Point(0, 0), p.valueAt(10000000000));
    EXPECT_EQ(Point(0, 0), p.valueAt(-10000000000));

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
    EXPECT_EQ(expected_point, p.valueAt(t_val));
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
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(-1, 1), p.valueAt(0), 1e-9));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(3.3, 7.8), p.valueAt(1), 1e-9));

    // Check that the intermediate point was interpolated correctly
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(0, 0), p.valueAt(0.5), 1e-9));

    // Check the coefficients are as expected
    // These were calculated using an online math tool
    // (https://www.wolframalpha.com/)
    EXPECT_NEAR(-1, p.getPolyX().getCoeff(0), 1e-9);
    EXPECT_NEAR(-0.3, p.getPolyX().getCoeff(1), 1e-9);
    EXPECT_NEAR(4.6, p.getPolyX().getCoeff(2), 1e-9);
    EXPECT_NEAR(1, p.getPolyY().getCoeff(0), 1e-9);
    EXPECT_NEAR(-10.8, p.getPolyY().getCoeff(1), 1e-9);
    EXPECT_NEAR(17.6, p.getPolyY().getCoeff(2), 1e-9);
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
