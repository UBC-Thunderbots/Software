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
        EXPECT_NEAR(expected.x, actual.x, 10e-6);
        EXPECT_NEAR(expected.y, actual.y, 10e-6);
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

TEST_F(Polynomial2dTest, get_value_2rd_order_polynomial)
{
    Polynomial2dOrder2_t poly = {
        .x = {.coefficients = {-2, 0.5, 6}},
        .y = {.coefficients = {.2, 7, 2}},
    };

    expectVector2dEqual({.x = 6.0, .y = 2.0},
                        shared_polynomial2d_getValueOrder2(poly, 0));
    expectVector2dEqual({.x = 6.03, .y = 2.702},
                        shared_polynomial2d_getValueOrder2(poly, 0.1));
    expectVector2dEqual({.x = 5.93, .y = 1.302},
                        shared_polynomial2d_getValueOrder2(poly, -0.1));
    expectVector2dEqual({.x = -189, .y = 92},
                        shared_polynomial2d_getValueOrder2(poly, 10));
    expectVector2dEqual({.x = -199, .y = -48},
                        shared_polynomial2d_getValueOrder2(poly, -10));
}


TEST_F(Polynomial2dTest, get_arc_length_parametrization_vertical_line_0_divisions)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {0, 0}},
        .y = {.coefficients = {1, 0.1}},
    };

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 2);

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

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 4);

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -3, 3, parametrization);

    static float expected_t_values_storage[4] = {-3, -1, 1, 3};
    static float expected_s_values_storage[4] = {0, 2, 4, 6};
    ArcLengthParametrization_t expected       = {.t_values = expected_t_values_storage,
                                           .s_values = expected_s_values_storage};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

TEST_F(Polynomial2dTest, get_arc_length_parametrization_for_complex_function)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1, 3, -0.5, 70.1}},
        .y = {.coefficients = {-0.2, 38.2, -3, -3}},
    };

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 17);

    shared_polynomial_getArcLengthParametrizationOrder3(poly, -6, 6, parametrization);

    // These values were calculated via numerical integration in GNU Octave with the
    // following program:
    //
    // clang-format off
    // ```
    // f=@(t) sqrt((0.1* t.^2 + 3*t.^1 - 0.5).^2 + (-0.2*t.^2 + 38.2*t.^1 - 3).^2)
    // for t = linspace(-6, 6, 17)
    //     disp(quadcc(f, -6, t));
    //     disp(",")
    // end
    // ```
    // clang-format on
    static float expected_t_values[17]  = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25,
                                          -1.5, -0.75, 0,    0.75,  1.5,  2.25,
                                          3,    3.75,  4.5,  5.25,  6};
    static float expected_s_values[17]  = {0,      168.5,  314.3,  437.58, 538.49, 617.19,
                                          673.86, 708.65, 721.73, 730.46, 760.33, 811.43,
                                          883.60, 976.68, 1090.5, 1224.9, 1379.8};
    ArcLengthParametrization_t expected = {.t_values = expected_t_values,
                                           .s_values = expected_s_values};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}


TEST_F(Polynomial2dTest, get_position_at_arc_length_on_straight_line_single_division)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    static float t_values[2]                              = {-2, 2};
    static float s_values[2]                              = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .s_values = s_values, .num_values = 2};

    expectVector2dEqual({.x = -1.9, .y = -1.9},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 0, arc_length_parametrization));
    expectVector2dEqual({.x = -0.9, .y = -0.9},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 1, arc_length_parametrization));
    expectVector2dEqual({.x = 0.1, .y = 0.1},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 2, arc_length_parametrization));
    expectVector2dEqual({.x = 1.1, .y = 1.1},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 3, arc_length_parametrization));
    expectVector2dEqual({.x = 2.1, .y = 2.1},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 4, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_position_at_arc_length_on_complex_line_multiple_divisions)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1, 3, -0.5, 70.1}},
        .y = {.coefficients = {-0.2, 38.2, -3, -3}},
    };

    // These values were calculated via numerical integration in GNU Octave with the
    // following program:
    //
    // clang-format off
    // ```
    // f=@(t) sqrt((0.1* t.^2 + 3*t.^1 - 0.5).^2 + (-0.2*t.^2 + 38.2*t.^1 - 3).^2)
    // for t = linspace(-6, 6, 17)
    //     disp(quadcc(f, -6, t));
    //     disp(",")
    // end
    // ```
    // clang-format on
    //
    // However for the purposes of this test it really doesn't matter if they're
    // accurate or not, since what we're testing is that we perform the lookup correctly
    // in *whatever* the given arc length parametrization is.

    static float t_values[17] = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25, -1.5, -0.75, 0,
                                 0.75, 1.5,   2.25, 3,     3.75, 4.5,   5.25, 6};
    static float s_values[17] = {0,      168.5,  314.3,  437.58, 538.49, 617.19,
                                 673.86, 708.65, 721.73, 730.46, 760.33, 811.43,
                                 883.60, 976.68, 1090.5, 1224.9, 1379.8};
    ArcLengthParametrization_t arc_length_parametrization = {.t_values = t_values,
                                                             .s_values = s_values};

    // Check some arc length points that require no interpolation
    expectVector2dEqual({.x = 159.5, .y = 1433.40},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 0, arc_length_parametrization));
    expectVector2dEqual({.x = 70.1, .y = -3.0},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 1379.8, arc_length_parametrization));
    expectVector2dEqual({.x = 196.7, .y = 1311.0},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 721.73, arc_length_parametrization));

    // Check some arc length points that should be determined via interpolation
    expectVector2dEqual({.x = 128.06, .y = 871.02},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 126.38, arc_length_parametrization));
    expectVector2dEqual({.x = 120.03, .y = 736.41},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 345.12, arc_length_parametrization));
}
