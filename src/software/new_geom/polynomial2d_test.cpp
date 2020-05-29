#include "software/new_geom/polynomial2d.h"

#include <gtest/gtest.h>

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
