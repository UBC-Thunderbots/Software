#pragma once

#include <stdlib.h>
/**
 * A representation of a matrix that has an arbitrary number of rows and columns specified
 * by n_cols and n_rows. The recommended way to use this struct is to use the get_matrix
 * function, which will allocate the memory required for each of the rows in the matrix
 * and each of the entries in each of those rows. It will also assign the n_cols and
 * n_rows for the matrix. Then when you are done with the matrix, you will need to call
 * shared_matrix_destroy.
 */
typedef struct Matrix Matrix_t;

/**
 * Gets a matrix of the given size. This uses dynamic memory allocation so make sure to
 * call shared_matrix_destroy when you are done using it if you don't want memory leaks.
 *
 * @param n_rows [in] The number of rows in the matrix
 * @param n_cols [in] The number of columns in the matrix
 * @return Pointer to the created Matrix
 */
Matrix_t* shared_matrix_create(unsigned int n_rows, unsigned int n_cols);

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
 * @param A [in] the left matrix
 * @param B [in] the right matrix
 * @return the resulting Matrix from the matrix multiplication
 */
Matrix_t* shared_matrix_multiply(Matrix_t* A, Matrix_t* B);

/**
 * Creates a shared_matrix_transpose of the given in_matrix.
 *
 * @param in_matrix [in] the matrix to shared_matrix_transpose
 * @return the shared_matrix_transpose of the given matrix
 */
Matrix_t* shared_matrix_transpose(Matrix_t* in_matrix);

/**
 * Function sets the value of the matrix element at the specified row and column. The first row and column are index '1'.
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
void shared_matrix_setValueAtIndex(unsigned int row, unsigned int column, float value, Matrix_t* matrix);

/**
 * Function gets the value of the matrix element at the specified row and column, The first row and column are index '1'.
 *
 *
 * @pre row <= the number of rows in the matrix and row >= 1
 * @pre column <= the number of columns in the matrix and column >= 1
 *
 * @param row [in] The row of the matrix element
 * @param column [in] The column of the matrix element
 * @param matrix [in] The matrix
 *
 * @return The value of the matrix element at the specified index. If the pre conditions are not met, it will return zero.
 */
float shared_matrix_getValueAtIndex(unsigned int row, unsigned int column, Matrix_t* matrix);