#include "software/new_geom/polynomial.h"

#include <gtest/gtest.h>

TEST(PolynomialTest, test_default_constructor)
{
    Polynomial p;
    for (size_t i = 0; i < 10; i++)
    {
        EXPECT_DOUBLE_EQ(p.getCoeff(i), 0);
        EXPECT_DOUBLE_EQ(p.valueAt(i), 0);
    }
    EXPECT_DOUBLE_EQ(p.getOrder(), 0);
}

TEST(PolynomialTest, test_polynomial_flat_line_constructor)
{
    std::vector<double> coeffs({});
    Polynomial p(coeffs);
    EXPECT_EQ(p.getCoeff(0), 0);
    EXPECT_EQ(p.valueAt(1), 0);
    EXPECT_EQ(p.valueAt(-2), 0);
    EXPECT_EQ(p.valueAt(3), 0);
}

TEST(PolynomialTest, test_polynomial_coeffs_constructor)
{
    std::vector<double> coeffs({1, 2, 3});  // 1 + 2x + 3x^2
    Polynomial p(coeffs);
    EXPECT_DOUBLE_EQ(p.getCoeff(0), 1);
    EXPECT_DOUBLE_EQ(p.getCoeff(1), 2);
    EXPECT_DOUBLE_EQ(p.getCoeff(2), 3);
    EXPECT_DOUBLE_EQ(p.getCoeff(3), 0);

    EXPECT_DOUBLE_EQ(p.valueAt(-2), 9);
    EXPECT_DOUBLE_EQ(p.valueAt(1), 6);
    EXPECT_DOUBLE_EQ(p.valueAt(3), 34);
}

TEST(PolynomialTest, test_polynomial_coeffs_list_constructor)
{
    std::vector<double> coeffs({1, 2, 3});  // 1 + 2x + 3x^2
    Polynomial p({1, 2, 3});
    EXPECT_DOUBLE_EQ(p.getCoeff(0), 1);
    EXPECT_DOUBLE_EQ(p.getCoeff(1), 2);
    EXPECT_DOUBLE_EQ(p.getCoeff(2), 3);
    EXPECT_DOUBLE_EQ(p.getCoeff(3), 0);

    EXPECT_DOUBLE_EQ(p.valueAt(-2), 9);
    EXPECT_DOUBLE_EQ(p.valueAt(1), 6);
    EXPECT_DOUBLE_EQ(p.valueAt(3), 34);
}

TEST(PolynomialTest, test_set_coeff)
{
    std::vector<double> coeffs({1, 2, 3});  // 1 + 2x + 3x^2
    Polynomial p(coeffs);
    p.setCoeff(2, 4);
    p.setCoeff(3, 5);
    p.setCoeff(100, 6);
    EXPECT_DOUBLE_EQ(p.getCoeff(2), 4);
    EXPECT_DOUBLE_EQ(p.getCoeff(3), 5);
    EXPECT_DOUBLE_EQ(p.getCoeff(100), 6);
}

TEST(PolynomialTest, get_order)
{
    std::vector<double> coeffs({1, 2, 3});  // 1 + 2x + 3x^2
    Polynomial p(coeffs);
    EXPECT_EQ(p.getOrder(), 2);

    p.setCoeff(2, 0);
    EXPECT_EQ(p.getOrder(), 1);
}

TEST(PolynomialTest, test_get_order)
{
    std::vector<double> coeffs({1, 2, 3});  // 1 + 2x + 3x^2
    Polynomial p(coeffs);
    EXPECT_DOUBLE_EQ(p.getOrder(), 2);

    p.setCoeff(100, 6);
    EXPECT_DOUBLE_EQ(p.getOrder(), 100);
}

TEST(PolynomialTest, test_plus_operator)
{
    Polynomial p1({2, 4.5, 6, 8});      // 2 + 4.5x + 6x^2 + 8x^3
    Polynomial p2({0, 3, 6, 9.2, 12});  // 0 + 3x + 6x^2 + 9.2x^3 + 12x^4
    Polynomial sum = p1 + p2;
    EXPECT_DOUBLE_EQ(sum.getCoeff(0), 2);
    EXPECT_DOUBLE_EQ(sum.getCoeff(1), 7.5);
    EXPECT_DOUBLE_EQ(sum.getCoeff(2), 12);
    EXPECT_DOUBLE_EQ(sum.getCoeff(3), 17.2);
    EXPECT_DOUBLE_EQ(sum.getCoeff(4), 12);
    EXPECT_DOUBLE_EQ(sum.getCoeff(5), 0);
    EXPECT_DOUBLE_EQ(sum.getOrder(), 4);
}

TEST(PolynomialTest, test_minus_operator)
{
    Polynomial p1({2, 4.5, 6, 8});      // 2 + 4.5x + 6x^2 + 8x^3
    Polynomial p2({0, 3, 6, 9.2, 12});  // 0 + 3x + 6x^2 + 9.2x^3 + 12x^4
    Polynomial difference = p1 - p2;
    EXPECT_DOUBLE_EQ(difference.getCoeff(0), 2);
    EXPECT_DOUBLE_EQ(difference.getCoeff(1), 1.5);
    EXPECT_DOUBLE_EQ(difference.getCoeff(2), 0);
    EXPECT_DOUBLE_EQ(difference.getCoeff(3), -1.2);
    EXPECT_DOUBLE_EQ(difference.getCoeff(4), -12);
    EXPECT_DOUBLE_EQ(difference.getCoeff(5), 0);
    EXPECT_DOUBLE_EQ(difference.getOrder(), 4);
}

TEST(PolynomialTest, test_multiplication_operator)
{
    Polynomial p1({2, 4.5, 6, 8});      // 2 + 4.5x + 6x^2 + 8x^3
    Polynomial p2({0, 3, 6, 9.2, 12});  // 0 + 3x + 6x^2 + 9.2x^3 + 12x^4
    Polynomial product = p1 * p2;
    EXPECT_DOUBLE_EQ(product.getCoeff(0), 0);
    EXPECT_DOUBLE_EQ(product.getCoeff(1), 6);
    EXPECT_DOUBLE_EQ(product.getCoeff(2), 25.5);
    EXPECT_DOUBLE_EQ(product.getCoeff(3), 63.4);
    EXPECT_DOUBLE_EQ(product.getCoeff(4), 125.4);
    EXPECT_DOUBLE_EQ(product.getCoeff(5), 157.2);
    EXPECT_DOUBLE_EQ(product.getCoeff(6), 145.6);
    EXPECT_DOUBLE_EQ(product.getCoeff(7), 96);
    EXPECT_DOUBLE_EQ(product.getCoeff(8), 0);
    EXPECT_DOUBLE_EQ(product.getOrder(), 7);
}

TEST(PolynomialTest, test_plus_equals_oeprator)
{
    Polynomial p1({4, 2.3, 1, 6, 2});     // 4 + 2.3x + 1x^2 + 6x^3 + 2x^4
    Polynomial p2({7, 2, 3, 8.3, 1, 5});  // 7 + 2x + 3x^2 + 8.3x^3 + 1x^4 + 5x^5
    p1 += p2;
    EXPECT_DOUBLE_EQ(p1.getCoeff(0), 11);
    EXPECT_DOUBLE_EQ(p1.getCoeff(1), 4.3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(2), 4);
    EXPECT_DOUBLE_EQ(p1.getCoeff(3), 14.3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(4), 3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(5), 5);
    EXPECT_DOUBLE_EQ(p1.getCoeff(6), 0);
    EXPECT_DOUBLE_EQ(p1.getOrder(), 5);
}

TEST(PolynomialTest, test_minus_equals_oeprator)
{
    Polynomial p1({4, 2.3, 1, 6, 2});     // 4 + 2.3x + 1x^2 + 6x^3 + 2x^4
    Polynomial p2({7, 2, 3, 8.3, 1, 5});  // 7 + 2x + 3x^2 + 8.3x^3 + 1x^4 + 5x^5
    p1 -= p2;
    EXPECT_DOUBLE_EQ(p1.getCoeff(0), -3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(1), 0.3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(2), -2);
    EXPECT_DOUBLE_EQ(p1.getCoeff(3), -2.3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(4), 1);
    EXPECT_DOUBLE_EQ(p1.getCoeff(5), -5);
    EXPECT_DOUBLE_EQ(p1.getCoeff(6), 0);
    EXPECT_DOUBLE_EQ(p1.getOrder(), 5);
}

TEST(PolynomialTest, test_multiply_equals_oeprator)
{
    Polynomial p1({4, 2.3, 1, 6, 2});     // 4 + 2.3x + 1x^2 + 6x^3 + 2x^4
    Polynomial p2({7, 2, 3, 8.3, 1, 5});  // 7 + 2x + 3x^2 + 8.3x^3 + 1x^4 + 5x^5
    p1 *= p2;
    EXPECT_DOUBLE_EQ(p1.getCoeff(0), 28);
    EXPECT_DOUBLE_EQ(p1.getCoeff(1), 24.1);
    EXPECT_DOUBLE_EQ(p1.getCoeff(2), 23.6);
    EXPECT_DOUBLE_EQ(p1.getCoeff(3), 84.1);
    EXPECT_DOUBLE_EQ(p1.getCoeff(4), 52.09);
    EXPECT_DOUBLE_EQ(p1.getCoeff(5), 52.6);
    EXPECT_DOUBLE_EQ(p1.getCoeff(6), 68.3);
    EXPECT_DOUBLE_EQ(p1.getCoeff(7), 27.6);
    EXPECT_DOUBLE_EQ(p1.getCoeff(8), 32);
    EXPECT_DOUBLE_EQ(p1.getCoeff(9), 10);
    EXPECT_DOUBLE_EQ(p1.getCoeff(10), 0);
    EXPECT_DOUBLE_EQ(p1.getOrder(), 9);
}
