#include "software/geom/polynomial.h"

#include <gtest/gtest.h>

TEST(TestSpline, test_polynomial_coeffs_constructor)
{
    std::vector<double> coeffs({1, 2, 3});
    Polynomial p(coeffs);
    EXPECT_EQ(p.getCoeffs(), coeffs);
    EXPECT_EQ(p.valueAt(1), 6);
    EXPECT_EQ(p.valueAt(-2), 3);
    EXPECT_EQ(p.valueAt(3), 18);
}

TEST(TestSpline, test_polynomial_coeffs_list_constructor)
{
    std::vector<double> coeffs({1, 2, 3});
    Polynomial p({1, 2, 3});
    EXPECT_EQ(p.getCoeffs(), coeffs);
    EXPECT_EQ(p.valueAt(1), 6);
    EXPECT_EQ(p.valueAt(-2), 3);
    EXPECT_EQ(p.valueAt(3), 18);
}

TEST(TestSpline, test_polynomial_value_pair_constructor)
{
    std::pair<double, double> constraint1, constraint2;
    constraint1 = std::make_pair(2.0, 3.0);
    constraint2 = std::make_pair(6.0, 4.0);
    std::vector<double> coeffs({.25, 2.5});
    Polynomial p(constraint1, constraint2);
    EXPECT_EQ(p.getCoeffs(), coeffs);
    EXPECT_EQ(p.valueAt(constraint1.first), constraint1.second);
    EXPECT_EQ(p.valueAt(constraint2.first), constraint2.second);
}

TEST(TestSpline, test_polynomial_invalid_value_pair_constructor)
{
    std::pair<double, double> constraint1, constraint2;
    constraint1 = std::make_pair(2.0, -3.0);
    constraint2 = std::make_pair(2.0, -4.0);
    try
    {
        Polynomial p(constraint1, constraint2);
    }
    catch (std::invalid_argument &e)
    {
        SUCCEED();
        return;
    }
    ADD_FAILURE() << "Successfully able to build a polynomial that isn't a function";
}

TEST(TestSpline, test_polynomial_flat_line_constructor)
{
    std::vector<double> coeffs({});
    Polynomial p(coeffs);
    EXPECT_EQ(p.getCoeffs(), coeffs);
    EXPECT_EQ(p.valueAt(1), 0);
    EXPECT_EQ(p.valueAt(-2), 0);
    EXPECT_EQ(p.valueAt(3), 0);
}
