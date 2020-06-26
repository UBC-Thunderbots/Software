extern "C"
{
#include "firmware/shared/math/polynomial_1d.h"
}

#include <gtest/gtest.h>

TEST(Polynomial1dTest, get_polynomial_value_from_0th_degree_poly)
{
    Polynomial1dOrder0_t poly = {.coefficients = {2}};

    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder0(poly, 0), 10e-7);
    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder0(poly, 0.1f), 10e-7);
    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder0(poly, -0.1f), 10e-7);
    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder0(poly, 10), 10e-7);
    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder0(poly, -10), 10e-7);
}

TEST(Polynomial1dTest, get_polynomial_value_from_1st_degree_poly)
{
    Polynomial1dOrder1_t poly = {.coefficients = {0.5, 10}};

    EXPECT_NEAR(10, shared_polynomial1d_getValueOrder1(poly, 0), 10e-7);
    EXPECT_NEAR(10.05, shared_polynomial1d_getValueOrder1(poly, 0.1f), 10e-7);
    EXPECT_NEAR(9.95, shared_polynomial1d_getValueOrder1(poly, -0.1f), 10e-7);
    EXPECT_NEAR(15, shared_polynomial1d_getValueOrder1(poly, 10), 10e-7);
    EXPECT_NEAR(5, shared_polynomial1d_getValueOrder1(poly, -10), 10e-7);
}

TEST(Polynomial1dTest, get_polynomial_value_from_2nd_degree_poly)
{
    Polynomial1dOrder2_t poly = {.coefficients = {0.5, 7, 2}};

    EXPECT_NEAR(2, shared_polynomial1d_getValueOrder2(poly, 0), 10e-7);
    EXPECT_NEAR(2.705, shared_polynomial1d_getValueOrder2(poly, 0.1f), 10e-7);
    EXPECT_NEAR(1.305, shared_polynomial1d_getValueOrder2(poly, -0.1f), 10e-7);
    EXPECT_NEAR(122, shared_polynomial1d_getValueOrder2(poly, 10), 10e-7);
    EXPECT_NEAR(-18, shared_polynomial1d_getValueOrder2(poly, -10), 10e-7);
}

TEST(Polynomial1dTest, differentiate_1st_degree_poly)
{
    Polynomial1dOrder1_t poly  = {.coefficients = {2, 3}};
    Polynomial1dOrder0_t deriv = shared_polynomial1d_differentiateOrder1(poly);

    EXPECT_NEAR(2, deriv.coefficients[0], 10e-7);
}

TEST(Polynomial1dTest, differentiate_2nd_degree_poly)
{
    Polynomial1dOrder2_t poly  = {.coefficients = {2, 3, 4}};
    Polynomial1dOrder1_t deriv = shared_polynomial1d_differentiateOrder2(poly);

    EXPECT_NEAR(4, deriv.coefficients[0], 10e-7);
    EXPECT_NEAR(3, deriv.coefficients[1], 10e-7);
}

TEST(Polynomial1dTest, differentiate_3rd_degree_poly)
{
    Polynomial1dOrder3_t poly  = {.coefficients = {2, 3, 4, 5}};
    Polynomial1dOrder2_t deriv = shared_polynomial1d_differentiateOrder3(poly);

    EXPECT_NEAR(6, deriv.coefficients[0], 10e-7);
    EXPECT_NEAR(6, deriv.coefficients[1], 10e-7);
    EXPECT_NEAR(4, deriv.coefficients[2], 10e-7);
}
