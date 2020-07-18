#pragma once

/**
 * \brief Multiplies a matrix by a vector.
 *
 * \param[out] lhs the result, \p matrix * \p rhs
 * \param[in] lhs_len the size of the output vector
 * \param[in] rhs the vector to multiply
 * \param[in] rhs_len the size of the input vector
 * \param[in] matrix the matrix to multiply
 */
void matrix_mult(float* lhs, int lhs_len, const float* rhs, int rhs_len,
                 const float matrix[lhs_len][rhs_len]);

/**
 * \brief Multiplies a matrix's transpose by a vector.
 *
 * \param[out] lhs the result, \p matrixT * \p rhs
 * \param[in] lhs_len the size of the output vector
 * \param[in] rhs the vector to multiply
 * \param[in] rhs_len the size of the input vector
 * \param[in] matrix the matrix to multiply
 */
void matrix_mult_t(float* lhs, int lhs_len, const float* rhs, int rhs_len,
                   const float matrix[lhs_len][rhs_len]);
