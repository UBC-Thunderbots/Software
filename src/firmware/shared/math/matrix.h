

/**
 * A representation of a matrix that has an arbitrary number of rows and columns specified
 * by n_cols and n_rows. The recommended way to use this struct is to use the get_matrix
 * function, which will allocate the memory required for each of the rows in the matrix
 * and each of the entries in each of those rows. It will also assign the n_cols and
 * n_rows for the matrix. Then when you are done with the matrix, you will need to call
 * free_matrix.
 */
typedef struct
{
    float **rows;
    int n_cols;
    int n_rows;
} Matrix;

/**
 * Gets a matrix of the given size. This uses dynamic memory allocation so make sure to
 * call free_matrix when you are done using it if you don't want memory leaks.
 *
 * @param n_rows the number of rows in the matrix
 * @param n_cols the number of columns in the matrix
 * @return a Matrix object
 */
Matrix create_matrix(int n_rows, int n_cols);

/**
 * Frees the memory used up by a matrix.
 *
 * @param matrix the matrix to free
 */
void free_matrix(Matrix matrix);

/**
 * Multiplies two matrices together and returns the resulting matrix, which is of size
 * A.n_rows x B.n_cols
 *
 * @param A the left matrix
 * @param B the right matrix
 * @return the resulting Matrix from the matrix multiplication
 */
Matrix matmul(Matrix A, Matrix B);

/**
 * Rotates the coordinate axis through the angle represented by the given
 * unit vector such that the given vector is in a new reference frame.
 * The unit vector should be of the form {cos(theta), sin(theta)},
 * where theta is the angle you want to rotate the axis by.
 *
 * @param vector the vector to rotate
 * @param unit_vector {cos(theta), sin(theta)} of the angle you want to rotate the axis by
 */
void rotate_axis_2D(float vector[2], const float unit_vector[2]);

/**
 * Rotates the given vector through the angle represented by the given
 * unit vector. The unit vector should be of the form {cos(theta), sin(theta)},
 * where theta is the angle you want to rotate the vector by.
 *
 * @param vector the vector to rotate
 * @param unit_vector {cos(theta), sin(theta)} of the angle you want to rotate the vector
 * by
 */
void rotate_vector_2D(float vector[2], const float unit_vector[2]);

/**
 * Creates a transpose of the given in_matrix.
 *
 * @param in_matrix the matrix to transpose
 * @return the transpose of the given matrix
 */
Matrix transpose(Matrix in_matrix);
