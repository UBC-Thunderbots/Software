extern "C"
{
#include "firmware/shared/math/polynomial_2d.h"

#include "firmware/shared/math/polynomial_1d.h"
}

#include <gtest/gtest.h>
#include <math.h>

class Polynomial2dTest : public testing::Test
{
   protected:
    virtual void SetUp(void) {}

    virtual void TearDown(void) {}

    static void expectRawArraysEq(float* expected, float* actual, size_t num_elements,
                                  double tolerance = 10e-10)
    {
        for (size_t i = 0; i < num_elements; i++)
        {
            EXPECT_NEAR(expected[i], actual[i], tolerance)
                << "expected[" << i << "] is " << expected[i] << " but actual[" << i
                << "] is " << actual[i];
        }
    }

    static void expectArcLengthParametrizationsEqual(ArcLengthParametrization_t expected,
                                                     ArcLengthParametrization_t actual,
                                                     double tolerance = 10e-10)
    {
        ASSERT_EQ(expected.num_values, actual.num_values);

        expectRawArraysEq(expected.t_values, actual.t_values, expected.num_values,
                          tolerance);
        expectRawArraysEq(expected.arc_length_values, actual.arc_length_values,
                          expected.num_values, tolerance);
    }

    static void expectVector2dEqual(Vector2d_t expected, Vector2d_t actual,
                                    double tolerance = 10e-6)
    {
        EXPECT_NEAR(expected.x, actual.x, tolerance);
        EXPECT_NEAR(expected.y, actual.y, tolerance);
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

TEST_F(Polynomial2dTest, get_derivative_3rd_order_polynomial)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {-2, 0.5, 6, 99}},
        .y = {.coefficients = {.2, 7, 2, 38}},
    };

    Polynomial2dOrder2_t derivative = shared_polynomial2d_differentiateOrder3(poly);

    EXPECT_FLOAT_EQ(-6, derivative.x.coefficients[0]);
    EXPECT_FLOAT_EQ(1, derivative.x.coefficients[1]);
    EXPECT_FLOAT_EQ(6, derivative.x.coefficients[2]);

    EXPECT_NEAR(0.6, derivative.y.coefficients[0], 10e-7);
    EXPECT_FLOAT_EQ(14, derivative.y.coefficients[1]);
    EXPECT_FLOAT_EQ(2, derivative.y.coefficients[2]);
}

TEST_F(Polynomial2dTest, get_arc_length_parametrization_num_values_0)
{
    // This function should probably never be used like this, but this checks that
    // we're at least executing defined behavior
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {0, 0}},
        .y = {.coefficients = {1, 0.1}},
    };

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 0);

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -2, 2, parametrization);

    float expected_t_values[0]          = {};
    float expected_arc_length_values[0] = {};
    ArcLengthParametrization_t expected = {
        .t_values          = expected_t_values,
        .arc_length_values = expected_arc_length_values,
        .num_values        = 0};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

TEST_F(Polynomial2dTest, get_arc_length_parametrization_num_values_1)
{
    // This function should probably never be used like this, but this checks that
    // we're at least executing defined behavior
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 1);

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -1, 2, parametrization);

    float expected_t_values[1]          = {0.5};
    float expected_arc_length_values[1] = {0};
    ArcLengthParametrization_t expected = {
        .t_values          = expected_t_values,
        .arc_length_values = expected_arc_length_values,
        .num_values        = 1};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}

TEST_F(Polynomial2dTest, get_arc_length_parametrization_vertical_line_0_divisions)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {0, 0}},
        .y = {.coefficients = {1, 0.1}},
    };

    CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(parametrization, 2);

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -2, 2, parametrization);

    float expected_t_values[2]          = {-2, 2};
    float expected_arc_length_values[2] = {0, 4};
    ArcLengthParametrization_t expected = {
        .t_values          = expected_t_values,
        .arc_length_values = expected_arc_length_values,
        .num_values        = 2};

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

    float expected_t_values_storage[4]          = {-3, -1, 1, 3};
    float expected_arc_length_values_storage[4] = {
        0.0, (float)sqrt(4 + 4), (float)sqrt(16 + 16), (float)sqrt(36 + 36)};
    ArcLengthParametrization_t expected = {
        .t_values          = expected_t_values_storage,
        .arc_length_values = expected_arc_length_values_storage,
        .num_values        = 4};

    expectArcLengthParametrizationsEqual(expected, parametrization);
}


TEST_F(Polynomial2dTest, get_arc_length_parametrization_for_complex_function)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1 / 3, 1.5, -0.5, 70.1}},
        .y = {.coefficients = {-0.2 / 3, 19.1, -3, -3}},
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
    float expected_t_values[17] = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25, -1.5, -0.75, 0,
                                   0.75, 1.5,   2.25, 3,     3.75, 4.5,   5.25, 6};
    float expected_arc_length_values[17] = {
        0,      168.5,  314.3,  437.58, 538.49, 617.19, 673.86, 708.65, 721.73,
        730.46, 760.33, 811.43, 883.60, 976.68, 1090.5, 1224.9, 1379.8};
    ArcLengthParametrization_t expected = {
        .t_values          = expected_t_values,
        .arc_length_values = expected_arc_length_values,
        .num_values        = 17};

    expectArcLengthParametrizationsEqual(expected, parametrization, 1);
}

TEST_F(Polynomial2dTest, get_position_at_arc_length_on_straight_line_1_values)
{
    // This function should probably never be used like this, but this checks that
    // we're at least executing defined behavior

    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.2}},
    };

    float t_values[1]                                     = {0.5};
    float arc_length_values[1]                            = {99};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 1};

    expectVector2dEqual({.x = 0.6, .y = 0.7},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 99, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_position_at_arc_length_on_straight_line_single_division)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

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

TEST_F(Polynomial2dTest, get_position_on_arc_length_above_arc_lengths_in_parametrization)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

    expectVector2dEqual({.x = 2.1, .y = 2.1},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, 5, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_position_on_arc_length_below_arc_lengths_in_parametrization)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

    expectVector2dEqual({.x = -1.9, .y = -1.9},
                        shared_polynomial2d_getPositionAtArcLengthOrder1(
                            poly, -5, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_position_at_arc_length_on_complex_line_multiple_divisions)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1 / 3, 1.5, -0.5, 70.1}},
        .y = {.coefficients = {-0.2 / 3, 19.1, -3, -3}},
    };

    // Note that for the purposes of this test it does not matter what the t and s
    // values here actually are, as long as they're both in ascending order, as this
    // function's job is merely to interpolate over the given set of values, whatever
    // those values might be.
    float t_values[17]          = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25, -1.5, -0.75, 0,
                          0.75, 1.5,   2.25, 3,     3.75, 4.5,   5.25, 6};
    float arc_length_values[17] = {0,      168.5,  314.3,  437.58, 538.49, 617.19,
                                   673.86, 708.65, 721.73, 730.46, 760.33, 811.43,
                                   883.60, 976.68, 1090.5, 1224.9, 1379.8};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 17};

    // Check some arc length points that require no interpolation
    expectVector2dEqual({.x = 119.9, .y = 717.00},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 0, arc_length_parametrization),
                        0.01);
    expectVector2dEqual({.x = 128.3, .y = 652.2},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 1379.8, arc_length_parametrization),
                        0.01);
    expectVector2dEqual({.x = 70.1, .y = -3.0},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 721.73, arc_length_parametrization),
                        0.01);

    // Check some arc length points that should be determined via interpolation
    expectVector2dEqual({.x = 111.81, .y = 588.75},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 126.38, arc_length_parametrization),
                        0.01);
    expectVector2dEqual({.x = 97.479, .y = 370.5},
                        shared_polynomial2d_getPositionAtArcLengthOrder3(
                            poly, 345.12, arc_length_parametrization),
                        0.01);
}

TEST_F(Polynomial2dTest, get_t_value_at_arc_length_on_straight_line_1_values)
{
    // This function should probably never be used like this, but this checks that
    // we're at least executing defined behavior

    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.2}},
    };

    float t_values[1]                                     = {0.5};
    float arc_length_values[1]                            = {99};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 1};

    EXPECT_FLOAT_EQ(0.5, shared_polynomial2d_getTValueAtArcLengthOrder1(
                             poly, 99, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_t_value_at_arc_length_on_straight_line_single_division)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

    EXPECT_FLOAT_EQ(-2, shared_polynomial2d_getTValueAtArcLengthOrder1(
                            poly, 0, arc_length_parametrization));
    EXPECT_FLOAT_EQ(-1, shared_polynomial2d_getTValueAtArcLengthOrder1(
                            poly, 1, arc_length_parametrization));
    EXPECT_FLOAT_EQ(0, shared_polynomial2d_getTValueAtArcLengthOrder1(
                           poly, 2, arc_length_parametrization));
    EXPECT_FLOAT_EQ(1, shared_polynomial2d_getTValueAtArcLengthOrder1(
                           poly, 3, arc_length_parametrization));
    EXPECT_FLOAT_EQ(2, shared_polynomial2d_getTValueAtArcLengthOrder1(
                           poly, 4, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_t_value_on_arc_length_above_arc_lengths_in_parametrization)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

    EXPECT_FLOAT_EQ(2, shared_polynomial2d_getTValueAtArcLengthOrder1(
                           poly, 5, arc_length_parametrization));
}


TEST_F(Polynomial2dTest, get_total_arc_length)
{
    Polynomial2dOrder1_t poly = {.x = {.coefficients = {4, 0}},
                                 .y = {.coefficients = {2, 0}}};

    // Create the parmeterization to contain the desired number of segments
    ArcLengthParametrization_t arc_length_parameterization;
    arc_length_parameterization.num_values = 3;

    float t_values[]          = {0, 1, 2};
    float arc_length_values[] = {0, 2, 20};

    arc_length_parameterization.t_values          = t_values;
    arc_length_parameterization.arc_length_values = arc_length_values;

    float total_arc_length =
        shared_polynomial2d_getTotalArcLength(arc_length_parameterization);

    EXPECT_EQ(total_arc_length, arc_length_values[2]);
}

TEST_F(Polynomial2dTest, get_t_value_on_arc_length_below_arc_lengths_in_parametrization)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {1, 0.1}},
        .y = {.coefficients = {1, 0.1}},
    };

    float t_values[2]                                     = {-2, 2};
    float arc_length_values[2]                            = {0, 4};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 2};

    EXPECT_FLOAT_EQ(-2, shared_polynomial2d_getTValueAtArcLengthOrder1(
                            poly, -5, arc_length_parametrization));
}

TEST_F(Polynomial2dTest, get_t_value_at_arc_length_on_complex_line_multiple_divisions)
{
    Polynomial2dOrder3_t poly = {
        .x = {.coefficients = {0.1 / 3, 1.5, -0.5, 70.1}},
        .y = {.coefficients = {-0.2 / 3, 19.1, -3, -3}},
    };

    // Note that for the purposes of this test it does not matter what the t and s
    // values here actually are, as long as they're both in ascending order, as this
    // function's job is merely to interpolate over the given set of values, whatever
    // those values might be.
    float t_values[17]          = {-6,   -5.25, -4.5, -3.75, -3.0, -2.25, -1.5, -0.75, 0,
                          0.75, 1.5,   2.25, 3,     3.75, 4.5,   5.25, 6};
    float arc_length_values[17] = {0,      168.5,  314.3,  437.58, 538.49, 617.19,
                                   673.86, 708.65, 721.73, 730.46, 760.33, 811.43,
                                   883.60, 976.68, 1090.5, 1224.9, 1379.8};
    ArcLengthParametrization_t arc_length_parametrization = {
        .t_values = t_values, .arc_length_values = arc_length_values, .num_values = 17};

    // Check some arc length points that require no interpolation
    EXPECT_FLOAT_EQ(-6, shared_polynomial2d_getTValueAtArcLengthOrder3(
                            poly, 0, arc_length_parametrization));
    EXPECT_FLOAT_EQ(6, shared_polynomial2d_getTValueAtArcLengthOrder3(
                           poly, 1379.8, arc_length_parametrization));
    EXPECT_FLOAT_EQ(0, shared_polynomial2d_getTValueAtArcLengthOrder3(
                           poly, 721.73, arc_length_parametrization));

    // Check some arc length points that should be determined via interpolation
    EXPECT_FLOAT_EQ(-5.4374776, shared_polynomial2d_getTValueAtArcLengthOrder3(
                                    poly, 126.38, arc_length_parametrization));
    EXPECT_FLOAT_EQ(-4.3125, shared_polynomial2d_getTValueAtArcLengthOrder3(
                                 poly, 345.12, arc_length_parametrization));
}
