#pragma once

#include <stdlib.h>
/**
 * A representation of a matrix that has an arbitrary number of rows and columns specified
 * by n_cols and n_rows. The recommended way to use this struct is to use the get_matrix
 * function, which will allocate the memory required for each of the rows in the matrix
 * and each of the entries in each of those rows. It will also assign the n_cols and
 * n_rows for the matrix.
 *
 * *******************************************************************
 * NOTE: Then when you are done with the matrix, you will need to call
 * shared_matrix_destroy(matrix).
 * ********************************************************************
 */
typedef struct Matrix Matrix_t;

/**
 * Creates a Matrix with the given numbers of rows and columns
 *
 * @param n_rows [in] The number of rows in the matrix
 * @param n_cols [in] The number of columns in the matrix
 *
 * @return Pointer to the created Matrix
 */
Matrix_t* shared_matrix_createBlank(unsigned int n_rows, unsigned int n_cols);

/**
 * Sets the values of a N*M matrix to be the values contained in the values array
 * NOTE: Value format is [ [ROW_1], [ROW_2], ROW_3]... [ROW_N] ]
 *
 * @param matrix [in] The N*M matrix to copy values into
 * @param values [in] The N*M array containing matrix values to be copied
 *
 * @return Pointer to the created Matrix
 */
void shared_matrix_setValues(Matrix_t* matrix, float** values);

/**
 * Frees the memory used up by a matrix.
 *
 * @param matrix [in] The matrix to destroy
 */
void shared_matrix_destroy(Matrix_t* matrix);

/**
 * Multiplies two matrices together and returns the resulting matrix, which is of size
 * A.n_rows x B.n_cols
 *
 * @param A [in] The left matrix
 * @param B [in] The right matrix
 *
 * @return the resulting Matrix from the matrix multiplication
 */
Matrix_t* shared_matrix_multiply(Matrix_t* A, Matrix_t* B);

/**
 * Creates a shared_matrix_transpose of the given in_matrix.
 *
 * @param in_matrix [in] The matrix to transpose
 *
 * @return the shared_matrix_transpose of the given matrix
 */
Matrix_t* shared_matrix_transpose(Matrix_t* in_matrix);

/**
 * Function sets the value of the matrix element at the specified row and column. The
 * first row and column are index '1'.
 *
 * NOTE: If any of the @pre conditions are not met, the function will do nothing
 * @pre row <= the number of rows in the matrix and row >= 1
 * @pre column <= the number of columns in the matrix and column >= 1
 *
 * @param row [in] The row of the matrix element
 * @param column [in] The column of the matrix element
 * @param value [in] The value to set
 * @param matrix [in] The matrix
 */
void shared_matrix_setValueAtIndex(unsigned int row, unsigned int column, float value,
                                   Matrix_t* matrix);

/**
 * Function gets the value of the matrix element at the specified row and column, The
 * first row and column are index '1'.
 *
 * @pre row <= the number of rows in the matrix and row >= 1
 * @pre column <= the number of columns in the matrix and column >= 1
 *
 * @param row [in] The row of the matrix element
 * @param column [in] The column of the matrix element
 * @param matrix [in] The matrix
 *
 * @return The value of the matrix element at the specified index. If the pre conditions
 * are not met, it will return zero.
 */
float shared_matrix_getValueAtIndex(unsigned int row, unsigned int column,
                                    Matrix_t* matrix);

/**
 * Function returns the number of rows in a Matrix
 *
 * @param matrix [in] The Matrix
 *
 * @return The number of rows in the matrix
 */
unsigned int shared_matrix_getNumRows(Matrix_t* matrix);

/**
 * Returns the number of columns in the Matrix
 *
 * @param matrix [in] The Matrix
 *
 * @return The number of columns in the Matrix
 */
unsigned int shared_matrix_getNumColumns(Matrix_t* matrix);

/**
 * Function inserts a specified row of values into a Matrix
 *
 * @pre The number of column values does not exceed the number of columns in the matrix
 * @pre The specified row number is equal to or less-than the number of rows in the matrix
 *
 * @param row [in] The row number to insert. The first row is index '1'
 * @param column_values [in] The value at each column in the row
 * @param num_columns [in] The number of columns in the matrix, this should be equal to
 * the number of elements in column_values
 * @param matrix [in/out] The Matrix that will be modified in-place to contain the new row
 * of values
 */
void shared_matrix_insertRow(unsigned int row, float column_values[],
                             unsigned int num_columns, Matrix_t* matrix);

/**
 * Function creates a Matrix and sets all elements. The dimensions of the Matrix will be
 * the same as the input array.
 *
 * @param matrix_values [in] The 2d array representing the matrix. matrix_values[x][y]
 * holds the value on the xth row and yth column.
 *
 * @param num_rows [in] The number of rows in the matrix
 *
 * @param num_columns [in] The number of columns in the matrix
 *
 * @return A pointer to the Matrix
 */
Matrix_t* shared_matrix_createMatrixFromValues(float** matrix_values,
                                               unsigned int num_rows,
                                               unsigned int num_columns);
