#include "firmware/main/math/polynomial_2d.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>

// TODO: consider combining all (or almost all) macros here

/**
 * Calculate the L2 norm of the vector at the given t-value
 *
 * @param p The polynomial to get the vector from
 * @param t The t-value to get the vector at
 * @return The L2 norm of the polynomial at the given t-value
 */
#define GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITIION(N)                          \
    float calculateL2NormAtValueOrder##N(Polynomial2dOrder##N##_t p, float t)            \
    {                                                                                    \
        const float x_value = shared_polynomial1d_getValueOrder##N(p.x, t);              \
        const float y_value = shared_polynomial1d_getValueOrder##N(p.y, t);              \
                                                                                         \
        return sqrt(pow(x_value, 2) + pow(y_value, 2));                                  \
    }
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITIION(3);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITIION(2);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITIION(1);
GENERATE_2D_POLYNOMIAL_CALCULATE_L2_NORM_DEFINITIION(0);


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
        Polynomial2dOrder##TO_ORDER##_t derivative = {                                   \
            .x = shared_polynomial1d_differentiateOrder##FROM_ORDER(p.x),                \
            .y = shared_polynomial1d_differentiateOrder##FROM_ORDER(p.y),                \
        };                                                                               \
        return derivative;                                                               \
    }
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(3, 2)
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(2, 1)
GENERATE_2D_POLYNOMIAL_DIFFERENTIATE_FUNCTION_DEFINITION(1, 0)


#define GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(         \
    N, N_minus_1)                                                                          \
    void shared_polynomial_getArcLengthParametrizationOrder##N(                            \
        Polynomial2dOrder##N##_t p, float t_min, float t_max,                              \
        ArcLengthParametrization_t parametrization)                                        \
    {                                                                                      \
        /* TODO: do we need this? remove and add test if not */                            \
        assert(t_min < t_max);                                                             \
                                                                                           \
        Polynomial2dOrder##N_minus_1##_t deriv =                                           \
            shared_polynomial2d_differentiateOrder##N(p);                                  \
                                                                                           \
        float dt = (t_max - t_min) / parametrization.num_values;                           \
                                                                                           \
        /* Populate the entries of the parametrization by numerically integrating the */   \
        /* derivative with Simpson's rule: */                                              \
        /* https://www.intmath.com/integration/6-simpsons-rule.php */                      \
        float initial_value        = calculateL2NormAtValueOrder##N_minus_1(deriv, t_min); \
        float sum_of_odd_s_values  = 0;                                                    \
        float sum_of_even_s_values = 0;                                                    \
        for (size_t i = 0; i < parametrization.num_values; i++)                            \
        {                                                                                  \
            const float t_i     = 2 * i * dt;                                              \
            const float value_i = calculateL2NormAtValueOrder##N_minus_1(deriv, t_i);      \
                                                                                           \
            const float t_i_plus_1 = (2 * i + 1) * dt;                                     \
            const float value_i_plus_1 =                                                   \
                calculateL2NormAtValueOrder##N_minus_1(deriv, t_i_plus_1);                 \
                                                                                           \
            const float s_i = initial_value + 4 * sum_of_odd_s_values +                    \
                              2 * sum_of_even_s_values + value_i;                          \
                                                                                           \
            parametrization.s_values[i] = s_i;                                             \
            parametrization.t_values[i] = t_i;                                             \
                                                                                           \
            sum_of_even_s_values += value_i;                                               \
            sum_of_odd_s_values += value_i_plus_1;                                         \
        }                                                                                  \
    }
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(3, 2)
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(2, 1)
GENERATE_2D_POLYNOMIAL_GET_ARC_LENGTH_PARAMETRIZATION_FUNCTION_DEFINITION(1, 0)

#define GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(N)         \
    Vector2d_t shared_polynomial2d_getPositionAtArcLengthOrder##N(                       \
        Polynomial2dOrder##N##_t p, float s,                                             \
        ArcLengthParametrization_t arc_length_parametrization)                           \
    {                                                                                    \
        /* Check that we have at least one s-value in the parametrization, and the s */  \
        /* value we're looking for is within the parametrization */                      \
        assert(arc_length_parametrization.num_values > 0);                               \
        assert(s >= arc_length_parametrization.s_values[0]);                             \
        assert(s <= arc_length_parametrization                                           \
                        .s_values[arc_length_parametrization.num_values - 1]);           \
                                                                                         \
        /* TODO: perform binary search and linear interpolation to get a t value */      \
        float t = 0;                                                                     \
                                                                                         \
        Vector2d_t result = {.x = shared_polynomial1d_getValueOrder##N(p.x, t),          \
                             .y = shared_polynomial1d_getValueOrder##N(p.y, t)};         \
                                                                                         \
        return result;                                                                   \
    }
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(3)
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(2)
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(1)
GENERATE_2D_POLYNOMIAL_GET_POSITION_AT_ARC_LENGTH_FUNCTION_DEFINITION(0)
