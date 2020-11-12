extern "C"
{
#include "firmware/shared/math/matrix.h"
}

#include <gtest/gtest.h>

TEST(MatrixTest, getAndSetMatrixValues)
{
    Matrix_t* matrix = shared_matrix_createBlank(3, 4);

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
    Matrix_t* matrix = shared_matrix_createBlank(3, 4);

    float row1[4] = {1, 2, 3, 4};
    float row2[4] = {5, 6, 7, 8};
    float row3[4] = {9, 10, 11, 12};

    shared_matrix_insertRow(1, row1, 4, matrix);
    shared_matrix_insertRow(2, row2, 4, matrix);
    shared_matrix_insertRow(3, row3, 4, matrix);

    Matrix_t* matrix2 = shared_matrix_createBlank(4, 2);

    float mat2_row1[2] = {13, 14};
    float mat2_row2[2] = {15, 16};
    float mat2_row3[2] = {17, 18};
    float mat2_row4[2] = {19, 20};
    shared_matrix_insertRow(1, mat2_row1, 2, matrix2);
    shared_matrix_insertRow(2, mat2_row2, 2, matrix2);
    shared_matrix_insertRow(3, mat2_row3, 2, matrix2);
    shared_matrix_insertRow(4, mat2_row4, 2, matrix2);
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

TEST(MatrixTest, testCreateMatrixFromValues)
{
    float matrix_values[3][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}};

    float* p_values[] = {matrix_values[0], matrix_values[1], matrix_values[2],
                         matrix_values[3]};

    Matrix_t* matrix = shared_matrix_createMatrixFromValues(p_values, 3, 4);

    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 1, matrix), 1);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 2, matrix), 2);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 3, matrix), 3);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 4, matrix), 4);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 1, matrix), 5);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 2, matrix), 6);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 3, matrix), 7);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 4, matrix), 8);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 1, matrix), 9);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 2, matrix), 10);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 3, matrix), 11);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 4, matrix), 12);

    shared_matrix_destroy(matrix);
}

TEST(MatrixTest, testCreateSetMatrixValues)
{
    float matrix_values[3][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}};

    float* p_values[] = {matrix_values[0], matrix_values[1], matrix_values[2],
                         matrix_values[3]};

    Matrix_t* matrix = shared_matrix_createMatrixFromValues(p_values, 3, 4);

    float matrix_values_new[3][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}};

    float* p_values_new[] = {matrix_values_new[0], matrix_values_new[1],
                             matrix_values_new[2], matrix_values_new[3]};
    shared_matrix_setValues(matrix, p_values_new);

    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 1, matrix), 1);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 2, matrix), 2);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 3, matrix), 3);
    EXPECT_EQ(shared_matrix_getValueAtIndex(1, 4, matrix), 4);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 1, matrix), 5);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 2, matrix), 6);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 3, matrix), 7);
    EXPECT_EQ(shared_matrix_getValueAtIndex(2, 4, matrix), 8);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 1, matrix), 9);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 2, matrix), 10);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 3, matrix), 11);
    EXPECT_EQ(shared_matrix_getValueAtIndex(3, 4, matrix), 12);

    shared_matrix_destroy(matrix);
}

TEST(MatrixTest, transposeMatrix)
{
    Matrix_t* matrix = shared_matrix_createBlank(2, 3);

    float row1[3] = {1, 2, 3};
    float row2[3] = {4, 5, 6};

    shared_matrix_insertRow(1, row1, 3, matrix);
    shared_matrix_insertRow(2, row2, 3, matrix);
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

TEST(MatrixTest, getColumnsAndRows)
{
    Matrix_t* matrix = shared_matrix_createBlank(2, 3);

    EXPECT_EQ(shared_matrix_getNumRows(matrix), 2);
    EXPECT_EQ(shared_matrix_getNumColumns(matrix), 3);

    shared_matrix_destroy(matrix);
}

TEST(MatrixTest, setRow)
{
    unsigned int num_rows    = 2;
    unsigned int num_columns = 3;

    float row[] = {4, 5, 6};

    Matrix_t* matrix = shared_matrix_createBlank(2, 3);
    shared_matrix_insertRow(2, row, num_columns, matrix);
    EXPECT_EQ(shared_matrix_getValueAtIndex(num_rows, 1, matrix), 4);
    EXPECT_EQ(shared_matrix_getValueAtIndex(num_rows, 2, matrix), 5);
    EXPECT_EQ(shared_matrix_getValueAtIndex(num_rows, 3, matrix), 6);

    shared_matrix_destroy(matrix);
}
