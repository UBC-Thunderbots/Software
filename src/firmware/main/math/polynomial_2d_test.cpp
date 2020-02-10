extern "C"
{
#include "firmware/main/math/polynomial_2d.h"
}

#include <gtest/gtest.h>

#include "polynomial_1d.h"
#include "polynomial_2d.h"

class Polynomial2dTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}

    static void expectRawArraysEq(float* expected, float* actual, size_t num_elements)
    {
        for (size_t i = 0; i < num_elements; i++)
        {
            EXPECT_EQ(expected[i], actual[i])
                << "expected[" << i << "] is " << expected[i] << "but actual[" << i
                << "] is " << actual[i];
        }
    }

    static void expectArcLengthParametrizationsEqual(ArcLengthParametrization_t expected,
                                                     ArcLengthParametrization_t actual)
    {
        // We can't put asserts in functions, so we have to settle for manually checking
        // and returning to avoid possible performing an out-of-bounds memory access
        EXPECT_EQ(expected.num_values, actual.num_values);
        if (expected.num_values != actual.num_values)
        {
            return;
        }

        expectRawArraysEq(expected.t_values, actual.t_values, expected.num_values);
        expectRawArraysEq(expected.s_values, actual.s_values, expected.num_values);
    }

    static void expectVector2dEqual(Vector2d_t expected, Vector2d_t actual)
    {
        EXPECT_NEAR(expected.x, actual.x, 10e-8);
        EXPECT_NEAR(expected.y, actual.y, 10e-8);
    }
};

TEST_F(Polynomial2dTest, get_value_0th_order_polynomial)
{
    Polynomial2dOrder0_t poly = {
        .x = {.coefficients = {-2}},
        .y = {.coefficients = {3}},
    };

    expectVector2dEqual({.x = -2, .y = 3}, shared_polynomial2d_getValueOrder0(poly, 0));
    expectVector2dEqual({.x = -2, .y = 3}, shared_polynomial2d_getValueOrder0(poly, 0.1));
    expectVector2dEqual({.x = -2, .y = 3},
                        shared_polynomial2d_getValueOrder0(poly, -0.1));
    expectVector2dEqual({.x = -2, .y = 3}, shared_polynomial2d_getValueOrder0(poly, 10));
    expectVector2dEqual({.x = -2, .y = 3}, shared_polynomial2d_getValueOrder0(poly, -10));
}

TEST_F(Polynomial2dTest, get_value_3rd_order_polynomial)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {-2, 0.5, 6}},
        .y = {.coefficients = {.2, 7, 2}},
    };

    expectVector2dEqual({.x = 6.0, .y = 2.0},
                        shared_polynomial2d_getValueOrder3(poly, 0));
    expectVector2dEqual({.x = 6.03, .y = 2.702},
                        shared_polynomial2d_getValueOrder3(poly, 0.1));
    expectVector2dEqual({.x = 5.93, .y = 1.302},
                        shared_polynomial2d_getValueOrder3(poly, -0.1));
    expectVector2dEqual({.x = -189, .y = 92},
                        shared_polynomial2d_getValueOrder3(poly, 10));
    expectVector2dEqual({.x = -199, .y = -48},
                        shared_polynomial2d_getValueOrder3(poly, -10));
}


TEST_F(Polynomial2dTest, get_arc_length_parametrization_vertical_line_0_divisions)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {0, 0}},
        .y = {.coefficients = {1, 0.1}},
    };

    static float t_values_storage[2];
    static float s_values_storage[2];
    ArcLengthParametrization_t parametrization = {
        .t_values   = t_values_storage,
        .s_values   = s_values_storage,
        .num_values = 2,
    };

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -2, 2, parametrization);

    static float expected_t_values_storage[2] = {-2, 2};
    static float expected_s_values_storage[2] = {0, 4};
    ArcLengthParametrization_t expected       = {.t_values = expected_t_values_storage,
                                           .s_values = expected_s_values_storage};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

TEST_F(Polynomial2dTest, get_arc_length_parametrization_diagonal_line_2_divisions)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    static float t_values_storage[4];
    static float s_values_storage[4];
    ArcLengthParametrization_t parametrization = {
        .t_values   = t_values_storage,
        .s_values   = s_values_storage,
        .num_values = 4,
    };

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -3, 3, parametrization);

    static float expected_t_values_storage[4] = {-3, -1, 1, 3};
    static float expected_s_values_storage[4] = {0, 2, 4, 6};
    ArcLengthParametrization_t expected       = {.t_values = expected_t_values_storage,
                                           .s_values = expected_s_values_storage};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

// Compare computed arc length values against those computed using Matlab/Octave for a
// significantly complex 2d function
TEST_F(Polynomial2dTest, get_arc_length_parametrization_for_complex_function)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1, 3, -0.5, 70.1}},
        .y = {.coefficients = {-0.2, 38.2, -3, -3}},
    };

    static float t_values_storage[17];
    static float s_values_storage[17];
    ArcLengthParametrization_t parametrization = {
        .t_values   = t_values_storage,
        .s_values   = s_values_storage,
        .num_values = 17,
    };

    shared_polynomial_getArcLengthParametrizationOrder3(poly, -6, 6, parametrization);

    // These values were calculated via integration in GNU Octave with the following
    // program:
    //
    // clang-format off
    //
    // f=@(t) sqrt((0.1* t.^3 + 3*t.^2 - 0.5*t + 70.1).^2 + (-0.2*t.^3 + 38.2*t.^2 - 3*t -3).^2)
    // for t = linspace(-6, 6, 17)
    //     quadcc(f, -6, t);
    //     disp(t)
    //     disp(",")
    // end
    //
    // clang-format on

    static float expected_t_values_storage[17] = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25,
                                                  -1.5, -0.75, 0,    0.75,  1.5,  2.25,
                                                  3,    3.75,  4.5,  5.25,  6};
    static float expected_s_values_storage[17] = {
        -6,      -5.2500, -4.5000, -3.7500, -3,     -2.2500, -1.5000, -0.75000, 0,
        0.75000, 1.5000,  2.2500,  3,       3.7500, 4.5000,  5.2500,  6};
    ArcLengthParametrization_t expected = {.t_values = expected_t_values_storage,
                                           .s_values = expected_s_values_storage};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

// Lookup arc length values on a straight line

// Lookup arc length values on a significantly complex line
