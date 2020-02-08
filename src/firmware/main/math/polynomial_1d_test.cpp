extern "C"
{
#include "firmware/main/math/polynomial_1d.h"
}

#include <gtest/gtest.h>

class Polynomial1dTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}
};

TEST_F(PolynomialTest, get_polynomial_value_from_0th_degree_poly)
{
    Polynomial1dOrder0_t poly = {.coefficients = {2}};

    EXPECT_EQ(2, shared_polynomial1d_getValueOrder0(poly, 0));
    EXPECT_EQ(2, shared_polynomial1d_getValueOrder0(poly, 0.1));
    EXPECT_EQ(2, shared_polynomial1d_getValueOrder0(poly, -0.1));
    EXPECT_EQ(2, shared_polynomial1d_getValueOrder0(poly, 10));
    EXPECT_EQ(2, shared_polynomial1d_getValueOrder0(poly, -10));
}

TEST_F(PolynomialTest, get_polynomial_value_from_1st_degree_poly)
{
    Polynomial1dOrder1_t poly = {.coefficients = {0.5, 10}};

    EXPECT_EQ(10, shared_polynomial1d_getValueOrder1(poly, 0));
    EXPECT_EQ(10.05, shared_polynomial1d_getValueOrder1(poly, 0.1));
    EXPECT_EQ(9.95, shared_polynomial1d_getValueOrder1(poly, -0.1));
    EXPECT_EQ(15, shared_polynomial1d_getValueOrder1(poly, 10));
    EXPECT_EQ(5, shared_polynomial1d_getValueOrder1(poly, -10));
}

TEST_F(PolynomialTest, get_polynomial_value_from_2nd_degree_poly)
{
    Polynomial1dOrder2_t poly = {.coefficients = {0.5, 7, 2}};

    EXPECT_EQ(2, shared_polynomial1d_getValueOrder2(poly, 0));
    EXPECT_EQ(2.705, shared_polynomial1d_getValueOrder2(poly, 0.1));
    EXPECT_EQ(1.305, shared_polynomial1d_getValueOrder2(poly, -0.1));
    EXPECT_EQ(122, shared_polynomial1d_getValueOrder2(poly, 10));
    EXPECT_EQ(-18, shared_polynomial1d_getValueOrder2(poly, -10));
}

TEST_F(PolynomialTest, differentiate_1st_degree_poly){
    Polynomial1dOrder1_t poly = {.coefficients = {2, 3}};
    Polynomial1dOrder0_t deriv = shared_polynomial1d_differentiateOrder1(poly);

    EXPECT_EQ(2, deriv.coefficients[0]);
}

TEST_F(PolynomialTest, differentiate_2nd_degree_poly){
    Polynomial1dOrder2_t poly = {.coefficients = {2, 3, 4}};
    Polynomial1dOrder1_t deriv = shared_polynomial1d_differentiateOrder2(poly);

    EXPECT_EQ(2, deriv.coefficients[0]);
    EXPECT_EQ(3, deriv.coefficients[1]);
}

TEST_F(PolynomialTest, differentiate_3rd_degree_poly){
    Polynomial1dOrder3_t poly = {.coefficients = {2, 3, 4, 5}};
    Polynomial1dOrder2_t deriv = shared_polynomial1d_differentiateOrder3(poly);

    EXPECT_EQ(2, deriv.coefficients[0]);
    EXPECT_EQ(3, deriv.coefficients[1]);
    EXPECT_EQ(4, deriv.coefficients[2]);
}

