#include "software/geom/polynomial.h"

#include <gtest/gtest.h>

TEST(TestSpline, test_polynomial_coeffs_constructor)
{
    std::vector<double> coeffs({1, 2, 3});
    Polynomial p(coeffs);
    EXPECT_EQ(p.getCoeffs(), coeffs);
}

TEST(TestSpline, test_polynomial_coeffs_list_constructor)
{
    std::vector<double> coeffs({1, 2, 3});
    Polynomial p({1, 2, 3});
    EXPECT_EQ(p.getCoeffs(), coeffs);
}

TEST(TestSpline, test_polynomial_value_pair_constructor)
{
    std::pair<double, double> p1, p2;
    p1 = std::make_pair(2.0, 3.0);
    p2 = std::make_pair(6.0, 4.0);
    std::vector<double> coeffs({2.5, .25});
    Polynomial p(p1, p2);
    EXPECT_EQ(p.getCoeffs(), coeffs);
}
