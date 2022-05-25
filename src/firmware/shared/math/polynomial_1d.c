#include "firmware/shared/math/polynomial_1d.h"

#include <math.h>

/**
 * Compute the value at the given point for the polynomial formed by the given coeffs
 * @param coefficients The coefficients that makeup the polynomial, with the highest
 *                     order coefficients at the start
 * @param num_coefficients The number of coefficients
 * @param x The point to get the polynomial value at
 * @return The polynomial value at the given point
 */
float shared_polynomial1d_getValue(const float* coefficients,
                                   const size_t num_coefficients, float x)
{
    float result = 0;
    for (size_t order = 0; order < num_coefficients; order++)
    {
        result +=
            (float)(coefficients[num_coefficients - 1 - order] * powf(x, (float)order));
    }
    return result;
}

/**
 * Calculate the derivative of the polynomial given by the given coefficients
 *
 * @param coefficients [in] The coefficients representing the polynomial to differentiate,
 *                          with the highest order ones first.
 * @param num_coefficients [in] The number of coefficients
 * @param derivative_coefficients [out] This will be populated with the coefficients
 *      representing the derivative of the polynomial represented by the given
 *      coefficients. Must be allocated to be at least num_coefficients-1 long
 */
void shared_polynomial1d_differentiate(const float* coefficients, size_t num_coefficients,
                                       float* derivative_coefficients)
{
    for (size_t i = 0; i < num_coefficients - 1; i++)
    {
        derivative_coefficients[i] = coefficients[i] * (float)(num_coefficients - i - 1);
    }
}

#define GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(N)                          \
    float shared_polynomial1d_getValueOrder##N(Polynomial1dOrder##N##_t p, float x)      \
    {                                                                                    \
        return shared_polynomial1d_getValue(p.coefficients, N + 1, x);                   \
    }

GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(0)
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(1)
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(2)
GENERATE_1D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(3)

#define GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(FROM_ORDER, TO_ORDER)   \
    Polynomial1dOrder##TO_ORDER##_t shared_polynomial1d_differentiateOrder##FROM_ORDER(  \
        Polynomial1dOrder##FROM_ORDER##_t p)                                             \
    {                                                                                    \
        _Static_assert(TO_ORDER == FROM_ORDER - 1, "TO_ORDER != FROM_ORDER-1");          \
        Polynomial1dOrder##TO_ORDER##_t derivative;                                      \
        shared_polynomial1d_differentiate(p.coefficients, FROM_ORDER + 1,                \
                                          derivative.coefficients);                      \
        return derivative;                                                               \
    }

GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(3, 2)
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(2, 1)
GENERATE_1D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(1, 0)
