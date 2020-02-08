extern "C"
{
#include "firmware/main/math/polynomial.h"
}

#include <gtest/gtest.h>

class PolynomialTest : public testing::Test
{
   protected:
    virtual void SetUp() {}

    virtual void TearDown() {}
};

// Get polynomial value at zero

// Get polynomial value at several positions from a 5th degree polynomial

// Differentiate 0th degree
// Differentiate 1st degree
// Differentiate 2nd degree
// Differentiate 3rd degree

// TODO: move bits below to 2D polynomial tests

// Get arc length parametrization for a straight vertical line with a 1 division
// Get arc length parametrization for a straight vertical line with a 5 divisions

// Get arc length parametrization for a straight horizontal line with a 1 division
// Get arc length parametrization for a straight horizontal line with a 5 divisions

// Compare computed arc length values against those computed using Matlab/Octave
