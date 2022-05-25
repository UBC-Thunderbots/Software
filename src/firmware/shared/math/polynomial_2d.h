#pragma once

#include <stddef.h>

#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/vector_2d.h"

/**
 * This represents an arc length parametrization of a 2D polynomial via a list of t values
 * and s values, where a value at the same index in both lists indicates they are
 * equivalent.
 *
 * https://math.stackexchange.com/questions/751781/how-to-parametrize-a-curve-by-its-arc-length
 */
typedef struct ArcLengthParametrization
{
    // A list of t values that can be given to a 2D polynomial
    float* t_values;
    // A list of corresponding arc lengths along a 2D polynomial
    float* arc_length_values;
    // The size of both the t_values and arc_length_values lists
    size_t num_values;
} ArcLengthParametrization_t;

#define CREATE_STATIC_ARC_LENGTH_PARAMETRIZATION(NAME, NUM_VALUES)                       \
    static float ___##NAME##_t_values_storage[NUM_VALUES];                               \
    static float ___##NAME##_arc_length_values_storage[NUM_VALUES];                      \
    ArcLengthParametrization_t NAME = {                                                  \
        .t_values          = ___##NAME##_t_values_storage,                               \
        .arc_length_values = ___##NAME##_arc_length_values_storage,                      \
        .num_values        = NUM_VALUES}

/**
 * A 2d polynomial of order N represented as f(t) = <x(t), y(t)> where x(t) and y(t) are
 * 1d polynomials of order N.
 */
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
 * @pre t_min <= t_max
 *
 * @param p [in] The 2D polynomial to get the arc length parametrization of
 * @param t_min [in] The minimum t value
 * @param t_max [in] The maximum t value
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
 * Get an approximation of the `t` value from a given arclength
 *
 * This can be used to get the position at a given arclength
 * (ie. `p(t)` = (x_at_arclength, y_at_arclength)
 *
 * @pre s is between the minimum and maximum s value in the given parametrization
 * @pre arc_length_parametrization has at least one entry
 *
 * @param p [in] The polynomial from which to get the t-value at the given arc length
 * @param s [in] The arc length to get the t-value at. If this is above or below the
 *               min/max arc length in the given parametrization, it will be set to the
 *               closest arc length in the parametrization.
 * @param arc_length_parametrization [in] A re-parametrization of "p" in terms of arc
 *                                        length. *THIS MUST HAVE AT LEAST ONE ENTRY*
 *
 * @return An approximation of the t-value at the given arc length on the given
 *         polynomial
 */
#define GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DECLARATION(N)         \
    float shared_polynomial2d_getTValueAtArcLengthOrder##N(                              \
        Polynomial2dOrder##N##_t p, float s,                                             \
        ArcLengthParametrization_t arc_length_parametrization)

GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DECLARATION(1);
GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DECLARATION(2);
GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DECLARATION(3);

/**
 * Get an approximation of the position along the given polynomial at the given arc length
 *
 * @pre s is between the minimum and maximum s value in the given parametrization
 * @pre arc_length_parametrization has at least one entry
 *
 * @param p [in] The polynomial from which to get the position at the given arc length
 * @param s [in] The arc length to get the position at. If this is above or below the
 *               min/max arc length in the given parametrization, it will be set to the
 *               closest arc length in the parametrization.
 * @param arc_length_parametrization [in] A re-parametrization of "p" in terms of arc
 *                                        length. *THIS MUST HAVE AT LEAST ONE ENTRY*
 *
 * @return An approximation of the position at the given arc length on the given
 *         polynomial
 */
#define GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(N)        \
    Vector2d_t shared_polynomial2d_getPositionAtArcLengthOrder##N(                       \
        Polynomial2dOrder##N##_t p, float s,                                             \
        ArcLengthParametrization_t arc_length_parametrization)

GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(1);
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(2);
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DECLARATION(3);

/**
 * Returns a the total length of any arc specified by an arc parameterizaton
 *
 * @param arc_length_param [in] Arc length parameterization
 *
 * @return [out] The arc length in meters
 */
float shared_polynomial2d_getTotalArcLength(
    ArcLengthParametrization_t arc_length_paramameterization);

/**
 * Get value of the radius of curvature at any point along the 't' parameterized
 * polynomial2s
 *
 * NOTE: If the radius of curvature is infinite, FLT_MAX will be returned. No radius of
 * curvature greater than FLT_MAX can be calculated.
 *
 * @param p [in] The polynomial to be evaluated for radius of curvature
 *
 * @param t [in] The parameterizaton value the polynomial will be evaluated at
 *
 * @return The radius of curvature of the polynomial at 't' [units of distance]
 */
#define GENERATE_2D_POLYNOMIAL_GET_CURVATURE_AT_POSITION(ORDER, ORDER_MINUS_ONE,         \
                                                         ORDER_MINUS_TWO)                \
    float shared_polynomial2d_getCurvatureAtPositionOrder##ORDER(                        \
        Polynomial2dOrder##ORDER##_t p, float t)

GENERATE_2D_POLYNOMIAL_GET_CURVATURE_AT_POSITION(2, 1, 0);
GENERATE_2D_POLYNOMIAL_GET_CURVATURE_AT_POSITION(3, 2, 1);
