#include "matrix.h"

#include <stdlib.h>


Matrix create_matrix(int n_rows, int n_cols)
{
    Matrix matrix;
    matrix.rows   = (float **)malloc(sizeof(float *) * n_rows);
    matrix.n_rows = n_rows;
    matrix.n_cols = n_cols;
    int i;
    for (i = 0; i < n_rows; i++)
    {
        matrix.rows[i] = (float *)calloc((unsigned)n_cols, sizeof(float));
    }
    return matrix;
}

void free_matrix(Matrix matrix)
{
    int i, j;
    for (i = 0; i < matrix.n_rows; i++)
    {
        free(matrix.rows[i]);
    }
    free(matrix.rows);
}

Matrix matmul(Matrix A, Matrix B)
{
    int i;
    int j;
    int k;
    Matrix C = create_matrix(A.n_rows, B.n_cols);
    for (i = 0; i < B.n_cols; i++)
    {
        for (j = 0; j < A.n_rows; j++)
        {
            C.rows[j][i] = 0.0f;
            for (k = 0; k < A.n_cols; k++)
            {
                C.rows[j][i] += A.rows[j][k] * B.rows[k][i];
            }
        }
    }
    return C;
}

/**
 * Rotates the given vector by using the given rotation matrix.
 *
 * @param vector a 2D vector to rotate
 * @param rotation_matrix the rotation operator
 */
void do_rotation(float vector_array[2], Matrix rotation_matrix)
{
    Matrix vector     = create_matrix(2, 1);
    vector.rows[0][0] = vector_array[0];
    vector.rows[1][0] = vector_array[1];
    Matrix rotated    = matmul(rotation_matrix, vector);
    vector_array[0]   = rotated.rows[0][0];
    vector_array[1]   = rotated.rows[1][0];
    free_matrix(vector);
    free_matrix(rotated);
    free_matrix(rotation_matrix);
}

void rotate_axis_2D(float vector[2], const float unit_vector[2])
{
    Matrix rotation_matrix     = create_matrix(2, 2);
    rotation_matrix.rows[0][0] = unit_vector[0];
    rotation_matrix.rows[0][1] = unit_vector[1];
    rotation_matrix.rows[1][0] = -unit_vector[1];
    rotation_matrix.rows[1][1] = unit_vector[0];
    do_rotation(vector, rotation_matrix);
}

void rotate_vector_2D(float vector[2], const float unit_vector[2])
{
    Matrix rotation_matrix     = create_matrix(2, 2);
    rotation_matrix.rows[0][0] = unit_vector[0];
    rotation_matrix.rows[0][1] = -unit_vector[1];
    rotation_matrix.rows[1][0] = unit_vector[1];
    rotation_matrix.rows[1][1] = unit_vector[0];
    do_rotation(vector, rotation_matrix);
}


Matrix transpose(Matrix in_matrix)
{
    int i;
    int j;
    Matrix out_matrix = create_matrix(in_matrix.n_cols, in_matrix.n_rows);
    for (i = 0; i < in_matrix.n_rows; i++)
    {
        for (j = 0; j < in_matrix.n_cols; j++)
        {
            out_matrix.rows[j][i] = in_matrix.rows[i][j];
        }
    }
    return out_matrix;
}
