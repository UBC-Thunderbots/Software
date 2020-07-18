extern "C"
{
#include "firmware/shared/math/matrix.h"
}

#include <gtest/gtest.h>

TEST(MatrixTest, getAndSetMatrixValues)
{
    Matrix_t* matrix = shared_matrix_create(3, 4);

    unsigned int col = 2;
    unsigned int row = 3;
    float value      = -2.22f;
    shared_matrix_setValueAtIndex(row, col, value, matrix);

    EXPECT_EQ(shared_matrix_getValueAtIndex(row, col, matrix), value);

    shared_matrix_setValueAtIndex(row - 1, col + 1, value - 20, matrix);

    EXPECT_EQ(shared_matrix_getValueAtIndex(row - 1, col + 1, matrix), value - 20);

    shared_matrix_destroy(matrix);
}

TEST(MatrixTest, matrixMultiplication)
{
    Matrix_t* matrix = shared_matrix_create(3, 4);

    shared_matrix_setValueAtIndex(1, 1, 1, matrix);
    shared_matrix_setValueAtIndex(1, 2, 2, matrix);
    shared_matrix_setValueAtIndex(1, 3, 3, matrix);
    shared_matrix_setValueAtIndex(1, 4, 4, matrix);
    shared_matrix_setValueAtIndex(2, 1, 5, matrix);
    shared_matrix_setValueAtIndex(2, 2, 6, matrix);
    shared_matrix_setValueAtIndex(2, 3, 7, matrix);
    shared_matrix_setValueAtIndex(2, 4, 8, matrix);
    shared_matrix_setValueAtIndex(3, 1, 9, matrix);
    shared_matrix_setValueAtIndex(3, 2, 10, matrix);
    shared_matrix_setValueAtIndex(3, 3, 11, matrix);
    shared_matrix_setValueAtIndex(3, 4, 12, matrix);

    Matrix_t* matrix2 = shared_matrix_create(4, 2);
    shared_matrix_setValueAtIndex(1, 1, 13, matrix2);
    shared_matrix_setValueAtIndex(1, 2, 14, matrix2);
    shared_matrix_setValueAtIndex(2, 1, 15, matrix2);
    shared_matrix_setValueAtIndex(2, 2, 16, matrix2);
    shared_matrix_setValueAtIndex(3, 1, 17, matrix2);
    shared_matrix_setValueAtIndex(3, 2, 18, matrix2);
    shared_matrix_setValueAtIndex(4, 1, 19, matrix2);
    shared_matrix_setValueAtIndex(4, 2, 20, matrix2);

    Matrix_t* result = shared_matrix_multiply(matrix, matrix2);

    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 1, result), 170);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 2, result), 180);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 1, result), 426);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 2, result), 452);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 1, result), 682);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 2, result), 724);

    shared_matrix_destroy(matrix);
    shared_matrix_destroy(matrix2);
    shared_matrix_destroy(result);
}

TEST(MatrixTest, transposeMatrix)
{
    Matrix_t* matrix = shared_matrix_create(2, 3);

    shared_matrix_setValueAtIndex(1, 1, 1, matrix);
    shared_matrix_setValueAtIndex(1, 2, 2, matrix);
    shared_matrix_setValueAtIndex(1, 3, 3, matrix);
    shared_matrix_setValueAtIndex(2, 1, 4, matrix);
    shared_matrix_setValueAtIndex(2, 2, 5, matrix);
    shared_matrix_setValueAtIndex(2, 3, 6, matrix);

    Matrix_t* transpose = shared_matrix_transpose(matrix);

    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 1, transpose), 1.0f);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 1, transpose), 2.0f);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 1, transpose), 3.0f);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 2, transpose), 4.0f);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 2, transpose), 5.0f);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 2, transpose), 6.0f);

    shared_matrix_destroy(transpose);
    shared_matrix_destroy(matrix);
}