#pragma once

// Need this for size_t
#include <stddef.h>

#include "firmware/main/math/polynomial_1d.h"

// TODO: put this somewhere else
/**
 * A 2D Vector
 */
typedef struct Vector2d
{
    float x;
    float y;
} Vector2d_t;

// TODO: better name? This is specific to 2d polynomials
// TODO: jdoc
typedef struct ArcLengthParametrization
{
    // TODO: comemnts for each member
    float* t_values;
    float* s_values;
    size_t num_values;
} ArcLengthParametrization_t;

#define CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(NAME, NUM_VALUES)                       \
    static float ___##NAME##_t_values_storage[4];                                        \
    static float ___##NAME##_s_values_storage[4];                                        \
    ArcLengthParametrization_t NAME = {.t_values   = ___##NAME##_t_values_storage,       \
                                       .s_values   = ___##NAME##_s_values_storage,       \
                                       .num_values = NUM_VALUES}

/**
 * A 2d polynomial is represented as f(t) = <x(t), y(t)> where x(t) and y(t) are
 * 1d polynomials of order N.
 */
// TODO: better jdoc
#define GENERATE_2D_POLYNOMIAL_STRUCT_OF_ORDER(N)                                        \
    typedef struct Polynomial2dOrder##N                                                  \
    {                                                                                    \
        Polynomial1dOrder##N##_t x;                                                      \
        Polynomial1dOrder##N##_t y;                                                      \
    } Polynomial2dOrder##N##_t

GENERATE_2D_POLYNOMIAL_STRUCT_OF_ORDER(0);
GENERATE_2D_POLYNOMIAL_STRUCT_OF_ORDER(1);
GENERATE_2D_POLYNOMIAL_STRUCT_OF_ORDER(2);
GENERATE_2D_POLYNOMIAL_STRUCT_OF_ORDER(3);

/**
 * Get the position on the given 2D polynomial at the given t value
 *
 * @param p The polynomial to get the position from
 * @param t The t-value to get the position at
 * @return The position at the given t-value
 */
#define GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(N)                         \
    Vector2d_t shared_polynomial2d_getValueOrder##N(Polynomial2dOrder##N##_t p, float t)

GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(0);
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(1);
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(2);
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DECLARATION(3);

/**
 * Differentiate the given 2D polynomial
 *
 * @param p [in] The polynomial to differentiate, with degrees n_x and n_y for the x and y
 *               polynomials respectively.
 * @return The derivative of P
 */
#define GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(FROM_ORDER, TO_ORDER)  \
    Polynomial2dOrder##TO_ORDER##_t shared_polynomial2d_differentiateOrder##FROM_ORDER(  \
        Polynomial2dOrder##FROM_ORDER##_t p)

GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(1, 0);
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(2, 1);
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DECLARATION(3, 2);

/**
 * Get a numerical approximation to the arc length parametrization of a given polynomial
 *
 * This will use Simpsons rule to numerically integrate the arc length formula.
 * https://en.wikipedia.org/wiki/Simpson%27s_rule
 *
 * TODO: do we need this pre? remove and add test if not
 * @pre t_min <= t_max
 *
 * @param p [in] The 2D polynomial to get the arc length parametrization of
 * @param t_min [in] The minimum t value
 * @param t_max [in] The maximum t value
 * TODO: better jdoc here
 * @param parametrization [out] The arc-length parametrization of the given curve. The
 *                              members of this must be allocated to be of at least
 *                              size "num_divisions". The "num_divisions" member will be
 *                              used as the number of segments in the numerical
 *                              integration scheme.
 */
#define GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_DECLARATION(N)             \
    void shared_polynomial_getArcLengthParametrizationOrder##N(                          \
        Polynomial2dOrder##N##_t p, float t_min, float t_max,                            \
        ArcLengthParametrization_t parametrization)

GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_DECLARATION(0);
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_DECLARATION(1);
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_DECLARATION(2);
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_DECLARATION(3);

/**
 * Get an approximation of the position along the given polynomial at the given arc length
 *
 * @pre s is between the minimum and maximum s value in the given parametrization
 *
 * @param p [in] The polynomial from which to get the position at the given arc length
 * @param s [in] The arc length to get the position at
 * @param arc_length_parametrization [in] A re-parametrization of "p" in terms of arc
 *                                        length. This must have at least one entry
 */
#define GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(N)        \
    Vector2d_t shared_polynomial2d_getPositionAtArcLengthOrder##N(                       \
        Polynomial2dOrder##N##_t p, float s,                                             \
        ArcLengthParametrization_t arc_length_parametrization)

GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(1);
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(2);
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(3);
