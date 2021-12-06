#pragma once

/**
 * \brief  A sufficiently small positive float.
 */
const float HRVO_EPSILON = 0.00001f;

/**
 * \brief      Computes the square of a float.
 * \param[in]  scalar  The float to be squared.
 * \return     The square of the float.
 */
inline float sqr(float scalar)
{
    return scalar * scalar;
}
