#include "matrix_test.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "check.h"
#include "main/util/matrix.h"
#include "test.h"

START_TEST(test_matmul_vectors)
{
    Matrix A     = create_matrix(1, 2);
    A.rows[0][0] = 5.0f;
    A.rows[0][1] = 3.0f;

    Matrix B     = create_matrix(2, 1);
    B.rows[0][0] = 4.0f;
    B.rows[1][0] = 6.0f;

    Matrix C = matmul(A, B);

    ck_assert_float_eq_tol(38.0f, C.rows[0][0], TOL);
    free_matrix(A);
    free_matrix(B);
    free_matrix(C);
}
END_TEST

START_TEST(test_matmul_different_size)
{
    int i, j;
    Matrix A            = create_matrix(2, 3);
    float A_array[2][3] = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 3; j++)
        {
            A.rows[i][j] = A_array[i][j];
        }
    }
    Matrix B            = create_matrix(3, 2);
    float B_array[3][2] = {{3.0f, 2.0f}, {4.0f, 5.0f}, {9.0f, 8.0f}};

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 2; j++)
        {
            B.rows[i][j] = B_array[i][j];
        }
    }
    float expected_C[2][2] = {{38.0f, 36.0f}, {86.0f, 81.0f}};
    Matrix C               = matmul(A, B);
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            ck_assert_float_eq_tol(expected_C[i][j], C.rows[i][j], TOL);
        }
    }
    free_matrix(A);
    free_matrix(B);
    free_matrix(C);
}
END_TEST

START_TEST(test_matmul_same_size)
{
    int i, j;
    Matrix A            = create_matrix(3, 3);
    float A_array[3][3] = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};

    Matrix B            = create_matrix(3, 3);
    float B_array[3][3] = {{3.0f, 2.0f, 1.0f}, {4.0f, 5.0f, 6.0f}, {9.0f, 8.0f, 7.0f}};

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            A.rows[i][j] = A_array[i][j];
            B.rows[i][j] = B_array[i][j];
        }
    }

    float expected_C[3][3] = {
        {38.0f, 36.0f, 34.0f}, {86.0f, 81.0f, 76.0f}, {134.0f, 126.0f, 118.0f}};
    Matrix C = matmul(A, B);
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            ck_assert_float_eq_tol(expected_C[i][j], C.rows[i][j], TOL);
        }
    }
}
END_TEST

START_TEST(test_rotate_axis_2D)
{
    float vector[2]      = {1.0f, 1.0f};
    float angle          = (float)M_PI / 4.0f;
    float unit_vector[2] = {cosf(angle), sinf(angle)};
    rotate_axis_2D(vector, unit_vector);
    ck_assert_float_eq_tol(sqrtf(2.0f), vector[0], TOL);
    ck_assert_float_eq_tol(0.0f, vector[1], TOL);
}
END_TEST

START_TEST(test_rotate_vector_2D)
{
    float vector[2]      = {1.0f, 1.0f};
    float angle          = (float)M_PI / 4.0f;
    float unit_vector[2] = {cosf(angle), sinf(angle)};
    rotate_vector_2D(vector, unit_vector);
    ck_assert_float_eq_tol(0.0f, vector[0], TOL);
    ck_assert_float_eq_tol(sqrtf(2.0f), vector[1], TOL);
}
END_TEST

START_TEST(test_transpose)
{
    int i, j;
    float in_matrix[3][2] = {{3.0f, 5.0f}, {4.0f, 10.0f}, {2.0f, 6.0f}};
    Matrix in             = create_matrix(3, 2);
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 2; j++)
        {
            in.rows[i][j] = in_matrix[i][j];
        }
    }
    float expected_result[2][3] = {{3.0f, 4.0f, 2.0f}, {5.0f, 10.0f, 6.0f}};
    Matrix out                  = transpose(in);
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 3; j++)
        {
            ck_assert_float_eq_tol(expected_result[i][j], out.rows[i][j], TOL);
        }
    }
    free_matrix(in);
    free_matrix(out);
}
END_TEST

START_TEST(test_matrix_struct)
{
    Matrix matrix     = create_matrix(2, 2);
    matrix.rows[0][0] = 1.0f;
    matrix.rows[0][1] = 2.0f;
    matrix.rows[1][0] = 3.0f;
    matrix.rows[1][1] = 4.0f;

    float expected[2][2] = {{1.0f, 2.0f}, {3.0f, 4.0f}};
    int i;
    int j;
    for (i = 0; i < matrix.n_rows; i++)
    {
        for (j = 0; j < matrix.n_cols; j++)
        {
            ck_assert_float_eq_tol(expected[i][j], matrix.rows[i][j], TOL);
        }
    }
    free_matrix(matrix);
}
END_TEST

void run_matrix_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("Matrix Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc = tcase_create("Core");
    // add the tests for this file here
    tcase_add_test(tc, test_matmul_vectors);
    tcase_add_test(tc, test_matmul_different_size);
    tcase_add_test(tc, test_matmul_same_size);
    tcase_add_test(tc, test_rotate_axis_2D);
    tcase_add_test(tc, test_rotate_vector_2D);
    tcase_add_test(tc, test_transpose);
    tcase_add_test(tc, test_matrix_struct);
    // run the tests
    run_test(tc, s);
}
