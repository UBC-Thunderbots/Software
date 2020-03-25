#pragma once
#include <stddef.h>

/**
 * A polynomial with a list of coefficients, with the highest order coefficients at
 * the start of the list, ie:
 * coefficients[0]*x^(order) + coefficients[1]*x^(order-1) + ... + coefficients[order-1]
 */
#define GENERATE_1D_POLYNOMIAL_STRUCT_OF_ORDER(N)                                        \
    typedef struct Polynomial1dOrder##N                                                  \
    {                                                                                    \
        float coefficients[N + 1];                                                       \
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
#define GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(FROM_ORDER, TO_ORDER)  \
    Polynomial1dOrder##TO_ORDER##_t shared_polynomial1d_differentiateOrder##FROM_ORDER(  \
        Polynomial1dOrder##FROM_ORDER##_t p)

GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(1, 0);
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(2, 1);
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(3, 2);
