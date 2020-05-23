#include "firmware/shared/math/polynomial_2d.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * Calculate the L2 norm of the vector at the given t-value
 *
 * @param p The polynomial to get the vector from
 * @param t The t-value to get the vector at
 * @return The L2 norm of the polynomial at the given t-value
 */
#define GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITION(N)                           \
    float calculateL2NormAtValueOrder##N(Polynomial2dOrder##N##_t p, float t)            \
    {                                                                                    \
        const float x_value = shared_polynomial1d_getValueOrder##N(p.x, t);              \
        const float y_value = shared_polynomial1d_getValueOrder##N(p.y, t);              \
                                                                                         \
        return sqrt(pow(x_value, 2.0) + pow(y_value, 2.0));                              \
    }
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITION(3);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITION(2);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITION(1);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITION(0);

#define GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(N)                          \
    Vector2d_t shared_polynomial2d_getValueOrder##N(Polynomial2dOrder##N##_t p, float t) \
    {                                                                                    \
        Vector2d_t result = {                                                            \
            .x = shared_polynomial1d_getValueOrder##N(p.x, t),                           \
            .y = shared_polynomial1d_getValueOrder##N(p.y, t),                           \
        };                                                                               \
        return result;                                                                   \
    }
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(3)
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(2)
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(1)
GENERATE_2D_POLYNOMIAL_GET_VALUE_FUNCTION_DEFINITION(0)

#define GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(FROM_ORDER, TO_ORDER)   \
    Polynomial2dOrder##TO_ORDER##_t shared_polynomial2d_differentiateOrder##FROM_ORDER(  \
        Polynomial2dOrder##FROM_ORDER##_t p)                                             \
    {                                                                                    \
        _Static_assert(TO_ORDER == FROM_ORDER - 1, "TO_ORDER != FROM_ORDER-1");          \
        Polynomial2dOrder##TO_ORDER##_t derivative = {                                   \
            .x = shared_polynomial1d_differentiateOrder##FROM_ORDER(p.x),                \
            .y = shared_polynomial1d_differentiateOrder##FROM_ORDER(p.y),                \
        };                                                                               \
        return derivative;                                                               \
    }
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(3, 2)
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(2, 1)
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(1, 0)


#define GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(       \
    N, N_minus_1)                                                                        \
    void shared_polynomial_getArcLengthParametrizationOrder##N(                          \
        Polynomial2dOrder##N##_t p, float t_min, float t_max,                            \
        ArcLengthParametrization_t parametrization)                                      \
    {                                                                                    \
        assert(t_min < t_max);                                                           \
                                                                                         \
        if (parametrization.num_values == 1)                                             \
        {                                                                                \
            parametrization.arc_length_values[0] = 0;                                    \
            parametrization.t_values[0]          = (t_max + t_min) / 2.0;                \
            return;                                                                      \
        }                                                                                \
                                                                                         \
        Polynomial2dOrder##N_minus_1##_t deriv =                                         \
            shared_polynomial2d_differentiateOrder##N(p);                                \
                                                                                         \
        /* We divide dt by two here because we want to ensure we get the number of */    \
        /* values that the user requested, but simpsons rule requires that we use */     \
        /* an even number of points */                                                   \
        float dt = (t_max - t_min) / ((parametrization.num_values - 1) * 2);             \
                                                                                         \
        /* Populate the entries of the parametrization by numerically integrating the */ \
        /* derivative with Simpson's rule: */                                            \
        /* https://www.intmath.com/integration/6-simpsons-rule.php */                    \
        float initial_value = calculateL2NormAtValueOrder##N_minus_1(deriv, t_min);      \
        float sum_of_odd_arc_length_values   = 0;                                        \
        float sum_of_even_arc_length_values  = 0;                                        \
        parametrization.arc_length_values[0] = 0;                                        \
        parametrization.t_values[0]          = t_min;                                    \
        for (size_t i = 1; i < parametrization.num_values; i++)                          \
        {                                                                                \
            const float t_i     = t_min + 2 * i * dt;                                    \
            const float value_i = calculateL2NormAtValueOrder##N_minus_1(deriv, t_i);    \
                                                                                         \
            const float t_i_minus_1 = t_min + (2 * i - 1) * dt;                          \
            const float value_i_minus_1 =                                                \
                calculateL2NormAtValueOrder##N_minus_1(deriv, t_i_minus_1);              \
            sum_of_odd_arc_length_values += value_i_minus_1;                             \
                                                                                         \
            const float s_i = initial_value + 4 * sum_of_odd_arc_length_values +         \
                              2 * sum_of_even_arc_length_values + value_i;               \
                                                                                         \
            parametrization.arc_length_values[i] = s_i;                                  \
            parametrization.t_values[i]          = t_i;                                  \
                                                                                         \
            sum_of_even_arc_length_values += value_i;                                    \
        }                                                                                \
        for (size_t i = 0; i < parametrization.num_values; i++)                          \
        {                                                                                \
            parametrization.arc_length_values[i] *= dt / 3;                              \
        }                                                                                \
    }
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(3, 2)
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(2, 1)
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(1, 0)

#define GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(N)         \
    Vector2d_t shared_polynomial2d_getPositionAtArcLengthOrder##N(                       \
        Polynomial2dOrder##N##_t p, float arc_length,                                    \
        ArcLengthParametrization_t arc_length_parametrization)                           \
    {                                                                                    \
        float t = shared_polynomial2d_getTValueAtArcLengthOrder##N(                      \
            p, arc_length, arc_length_parametrization);                                  \
        Vector2d_t result = {.x = shared_polynomial1d_getValueOrder##N(p.x, t),          \
                             .y = shared_polynomial1d_getValueOrder##N(p.y, t)};         \
        return result;                                                                   \
    }
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(3)
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(2)
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(1)

#define GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DEFINITION(N)          \
    float shared_polynomial2d_getTValueAtArcLengthOrder##N(                              \
        Polynomial2dOrder##N##_t p, float arc_length,                                    \
        ArcLengthParametrization_t arc_length_parametrization)                           \
    {                                                                                    \
        /* Check that we have at least one s-value in the parametrization, and the s */  \
        /* value we're looking for is within the parametrization */                      \
        assert(arc_length_parametrization.num_values > 0);                               \
                                                                                         \
        const float max_arc_length_value =                                               \
            arc_length_parametrization                                                   \
                .arc_length_values[arc_length_parametrization.num_values - 1];           \
        const float min_arc_length_value =                                               \
            arc_length_parametrization.arc_length_values[0];                             \
        if (arc_length > max_arc_length_value)                                           \
        {                                                                                \
            arc_length = max_arc_length_value;                                           \
        }                                                                                \
        else if (arc_length < min_arc_length_value)                                      \
        {                                                                                \
            arc_length = min_arc_length_value;                                           \
        }                                                                                \
                                                                                         \
                                                                                         \
        /* Binary search to find between which two s-values in the */                    \
        /* table the given s value is */                                                 \
                                                                                         \
        size_t upper = arc_length_parametrization.num_values - 1;                        \
        size_t lower = 0;                                                                \
        size_t pivot = lower + (int)floor((upper - lower) / 2);                          \
        float arc_length_below_pivot =                                                   \
            arc_length_parametrization.arc_length_values[pivot];                         \
        float arc_length_above_pivot =                                                   \
            arc_length_parametrization.arc_length_values[pivot + 1];                     \
        while (upper != lower && !(arc_length >= arc_length_below_pivot &&               \
                                   arc_length <= arc_length_above_pivot))                \
        {                                                                                \
            if (arc_length > arc_length_above_pivot)                                     \
            {                                                                            \
                lower = pivot;                                                           \
            }                                                                            \
            else                                                                         \
            {                                                                            \
                upper = pivot;                                                           \
            }                                                                            \
            pivot = lower + floor((upper - lower) / 2);                                  \
            arc_length_below_pivot =                                                     \
                arc_length_parametrization.arc_length_values[pivot];                     \
            arc_length_above_pivot =                                                     \
                arc_length_parametrization.arc_length_values[pivot + 1];                 \
        }                                                                                \
                                                                                         \
        /* Linearly interpolate between the two t-values above and below the s-value */  \
        /* we found */                                                                   \
        const float t_below_pivot = arc_length_parametrization.t_values[pivot];          \
        const float t_above_pivot = arc_length_parametrization.t_values[pivot + 1];      \
        const float arc_length_ratio =                                                   \
            (arc_length - arc_length_below_pivot) /                                      \
            (arc_length_above_pivot - arc_length_below_pivot);                           \
        const float t =                                                                  \
            t_below_pivot + (t_above_pivot - t_below_pivot) * arc_length_ratio;          \
        return t;                                                                        \
    }
GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DEFINITION(3)
GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DEFINITION(2)
GENERATE_2D_POLYNOMIAL_GET_T_VALUE_AT_ARC_LENGTH_FUNCTION_DEFINITION(1)

float shared_polynomial2d_getTotalArcLength(
    ArcLengthParametrization_t arc_length_paramameterization)
{
    return arc_length_paramameterization
        .arc_length_values[arc_length_paramameterization.num_values - 1];
}
