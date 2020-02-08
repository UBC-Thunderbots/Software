#pragma once

// Need this for size_t
#include <stddef.h>

/**
 * A polynomial is a list of floats
 * s.t. n = coeffs.size() == the degree of the polynomial
 * and of the form
 * p[0]*x^(n-1) + p[1]*x^(n-2) + ... + p[n-1]
 */
// TODO: better jdoc
#define GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(N)                                        \
    typedef struct Polynomial1dOrder##N                                                  \
    {                                                                                    \
        float coefficients[N];                                                           \
    } Polynomial1dOrder##N##_t

GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(0);
GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(1);
GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(2);
GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(3);

/**
 * Compute the y-value at a given x-value for a 1D polynomial
 *
 * @param p The polynomial to get the y-value from
 * @param x The x value to get the corresponding y-values for
 *
 * @return The y-value at the given x-value
 */
#define GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(N)                         \
    float shared_polynomial1d_getValueOrder##N(Polynomial1dOrder##N##_t p, float x)

GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(0);
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(1);
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(2);
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(3);

/**
 * Differentiates the given 1d polynomial
 *
 * @param p The polynomial to differentiate.
 * @return The derivative of the given polynomial
 */
#define GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION(FROM_ORDER, TO_ORDER)              \
    Polynomial1dOrder##TO_ORDER##_t shared_polynomial1d_differentiateOrder##FROM_ORDER(  \
        Polynomial1dOrder##FROM_ORDER##_t p)

GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION(1, 0);
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION(2, 1);
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION(3, 2);
