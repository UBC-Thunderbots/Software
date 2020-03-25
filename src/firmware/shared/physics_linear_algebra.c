#include "firmware/shared/physics_linear_algebra.h"

void matrix_mult(float* lhs, int lhs_len, const float* rhs, int rhs_len,
                 const float matrix[lhs_len][rhs_len])
{
    for (int j = 0; j < lhs_len; ++j)
    {
        lhs[j] = 0.0f;
        for (int i = 0; i < rhs_len; ++i)
        {
            lhs[j] += matrix[j][i] * rhs[i];
        }
    }
}

void matrix_mult_t(float* lhs, int lhs_len, const float* rhs, int rhs_len,
                   const float matrix[rhs_len][lhs_len])
{
    for (int j = 0; j < lhs_len; ++j)
    {
        lhs[j] = 0.0f;
        for (int i = 0; i < rhs_len; ++i)
        {
            lhs[j] += matrix[i][j] * rhs[i];
        }
    }
}
