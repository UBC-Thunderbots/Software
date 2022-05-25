#include "firmware/shared/math/matrix.h"

#include <assert.h>
#include <stdlib.h>

struct Matrix
{
    float** rows;
    unsigned int n_cols;
    unsigned int n_rows;
};

Matrix_t* shared_matrix_createBlank(unsigned int n_rows, unsigned int n_cols)
{
    Matrix_t* matrix = (Matrix_t*)malloc(sizeof(Matrix_t));

    matrix->rows   = (float**)malloc(sizeof(float*) * n_rows);
    matrix->n_rows = n_rows;
    matrix->n_cols = n_cols;

    for (unsigned int i = 0; i < n_rows; i++)
    {
        matrix->rows[i] = (float*)calloc(n_cols, sizeof(float));
    }
    return matrix;
}

void shared_matrix_setValues(Matrix_t* matrix, float** values)
{
    const unsigned int num_columns = matrix->n_cols;
    const unsigned int num_rows    = matrix->n_rows;

    for (unsigned int row = 1; row <= num_rows; row++)
    {
        shared_matrix_insertRow(row, values[row - 1], num_columns, matrix);
    }
}

void shared_matrix_destroy(Matrix_t* matrix)
{
    for (unsigned int i = 0; i < matrix->n_rows; i++)
    {
        free(matrix->rows[i]);
    }
    free(matrix->rows);
    free(matrix);
}

Matrix_t* shared_matrix_multiply(Matrix_t* A, Matrix_t* B)
{
    Matrix_t* return_matrix = shared_matrix_createBlank(A->n_rows, B->n_cols);
    for (unsigned int i = 0; i < B->n_cols; i++)
    {
        for (unsigned int j = 0; j < A->n_rows; j++)
        {
            return_matrix->rows[j][i] = 0.0f;
            for (unsigned int k = 0; k < A->n_cols; k++)
            {
                return_matrix->rows[j][i] += A->rows[j][k] * B->rows[k][i];
            }
        }
    }
    return return_matrix;
}

Matrix_t* shared_matrix_transpose(Matrix_t* in_matrix)
{
    Matrix_t* out_matrix =
        shared_matrix_createBlank(in_matrix->n_cols, in_matrix->n_rows);
    for (unsigned int i = 0; i < in_matrix->n_rows; i++)
    {
        for (unsigned int j = 0; j < in_matrix->n_cols; j++)
        {
            out_matrix->rows[j][i] = in_matrix->rows[i][j];
        }
    }
    return out_matrix;
}

void shared_matrix_setValueAtIndex(unsigned int row, unsigned int column, float value,
                                   Matrix_t* matrix)
{
    assert(row <= matrix->n_rows && row >= 1);
    assert(column <= matrix->n_cols && column >= 1);

    // If the row or column are out of the range of the array then return.
    // It is likely safer to do nothing than segfault
    if (row > matrix->n_rows || column > matrix->n_cols)
    {
        return;
    }
    matrix->rows[row - 1][column - 1] = value;
}

float shared_matrix_getValueAtIndex(unsigned int row, unsigned int column,
                                    Matrix_t* matrix)
{
    assert(row <= matrix->n_rows && row >= 1);
    assert(column <= matrix->n_cols && column >= 1);

    // If the row or column are out of the range of the array then return.
    // It is likely safer to do nothing than segfault
    if (row > matrix->n_rows || column > matrix->n_cols || row < 1 || column < 1)
    {
        return 0.0f;
    }

    return matrix->rows[row - 1][column - 1];
}

unsigned int shared_matrix_getNumRows(Matrix_t* matrix)
{
    return matrix->n_rows;
}

unsigned int shared_matrix_getNumColumns(Matrix_t* matrix)
{
    return matrix->n_cols;
}

void shared_matrix_insertRow(unsigned int row, float column_values[],
                             unsigned int num_columns, Matrix_t* matrix)
{
    for (unsigned int column = 1; column <= num_columns; column++)
    {
        shared_matrix_setValueAtIndex(row, column, column_values[column - 1], matrix);
    }
}

Matrix_t* shared_matrix_createMatrixFromValues(float** matrix_values,
                                               unsigned int num_rows,
                                               unsigned int num_columns)
{
    Matrix_t* matrix = shared_matrix_createBlank(num_rows, num_columns);

    for (unsigned int row = 1; row <= num_rows; row++)
    {
        shared_matrix_insertRow(row, matrix_values[row - 1], num_columns, matrix);
    }

    return matrix;
}
