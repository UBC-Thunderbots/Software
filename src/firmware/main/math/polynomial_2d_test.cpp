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
        EXPECT_EQ(expected.num_values, actual.num_values);

        expectRawArraysEq(expected.t_values, actual.t_values);
        expectRawArraysEq(expected.s_values, actual.s_values);
    }
};

TEST_F(Polynomial2dTest, get_arc_length_parametrization_vertical_line_1_division)
{
    Polynomial2dOrder1_t poly = {
        .x = {.coefficients = {0, 0}},
        .y = {.coefficients = {1, 0.1}},
    };

    static t_values_storage[3];
    static s_values_storage[3];
    ArcLengthParametrization_t parametrization = {
        .t_values   = &t_values,
        .s_values   = &s_values,
        .num_values = 2,
    };

    shared_polynomial_getArcLengthParametrizationOrder1(poly, -2, 2, parametrization);

    static expected_t_values_storage[3];
    static expected_s_values_storage[3];
    ArcLengthParametrization_t expected = {.t_values = [ -2, 2 ], .s_values = [kk]};

    // TODO: YOU ARE HERE - NEED TO FINISH THESE TESTS
}

// Get arc length parametrization for a straight vertical line with a 5 divisions

// Get arc length parametrization for a straight horizontal line with a 1 division
// Get arc length parametrization for a straight horizontal line with a 5 divisions

// Compare computed arc length values against those computed using Matlab/Octave for a
// significantly complex 2d function

// Lookup arc length values on a straight line

// Lookup arc length values on a significantly complex line
