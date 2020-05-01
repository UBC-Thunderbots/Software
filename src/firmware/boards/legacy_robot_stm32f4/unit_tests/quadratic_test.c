#include "main/util/quadratic.h"

#include <math.h>

#include "check.h"
#include "test.h"

// This is an M matrix that is used for multiple tests
// It is primarily related to the Q matrix from the optimization
// where Q = M.T * M
static float M[3][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}};

START_TEST(test_build_M_matrix)
{
    PhysBot pb = {.rot = {.disp = 0}, .major_vec = {1, 1}, .minor_vec = {1, 0}};
    dr_data_t state;
    state.angle = 0;
    float M[3][4];
    build_M_matrix(pb, state, M);
    // check wheel 1 of matrix
    ck_assert_float_eq_tol(-0.5f + sqrt(3.0f) / 2.0f, M[0][0], TOL);
    ck_assert_float_eq_tol(-0.5f, M[1][0], TOL);
    ck_assert_float_eq(1, M[2][0]);
    // check wheel 2 of matrix
    ck_assert_float_eq_tol(-0.5f - sqrt(3.0f) / 2.0f, M[0][1], TOL);
    ck_assert_float_eq_tol(-sqrt(3.0f) / 2.0f, M[1][1], TOL);
    ck_assert_float_eq(1, M[2][1]);
    // check wheel 3 of matrix
    ck_assert_float_eq_tol(0.5f + sqrt(3.0f) / 2.0f, M[0][2], TOL);
    ck_assert_float_eq_tol(0.5f, M[1][2], TOL);
    ck_assert_float_eq(1, M[2][2]);
    // check wheel 4 of matrix
    ck_assert_float_eq_tol(-0.5f + sqrt(3.0f) / 2.0f, M[0][3], TOL);
    ck_assert_float_eq_tol(sqrt(3.0f) / 2.0f, M[1][3], TOL);
    ck_assert_float_eq(1, M[2][3]);
}
END_TEST

START_TEST(test_transpose)
{
    float M_T[4][3];
    transpose_qp(M, M_T);
    float expected_result[4][3] = {{1, 5, 9}, {2, 6, 10}, {3, 7, 11}, {4, 8, 12}};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            ck_assert_float_eq(expected_result[i][j], M_T[i][j]);
        }
    }
}
END_TEST

START_TEST(test_build_Q_matrix)
{
    float M_T[4][3];
    float Q[4][4];
    transpose_qp(M, M_T);
    build_Q_matrix(M, M_T, Q);
    float expected_result[4][4] = {{107, 122, 137, 152},
                                   {122, 140, 158, 176},
                                   {137, 158, 179, 200},
                                   {152, 176, 200, 224}};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ck_assert_float_eq(expected_result[i][j], Q[i][j]);
        }
    }
}
END_TEST

START_TEST(test_build_c_matrix)
{
    float a_req[3] = {0.1, 0.5, 0.4};
    float c[4];
    build_c_matrix(a_req, M, c);
    float expected_result[4] = {6.2, 7.2, 8.2, 9.2};
    for (int i = 0; i < 4; i++)
    {
        ck_assert_float_eq_tol(2 * expected_result[i], c[i], TOL);
    }
}
END_TEST

START_TEST(quadratic_test)
{
    PhysBot pb = {
        .rot = {.disp = 30.0f * M_PI / 180.0f}, .major_vec = {1, 1}, .minor_vec = {1, 0}};
    float a_req[3] = {0.1, 0.5, 0.4};
    dr_data_t state;
    state.angle = 0;
    quad_optimize(pb, state, a_req);
}
END_TEST

/**
 * Test function manager for quadratic.c
 */
void run_quadratic_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("Qudratic Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc = tcase_create("Core");
    // add the tests for this file here
    tcase_add_test(tc, quadratic_test);
    tcase_add_test(tc, test_build_M_matrix);
    tcase_add_test(tc, test_transpose);
    tcase_add_test(tc, test_build_Q_matrix);
    tcase_add_test(tc, test_build_c_matrix);
    // run the tests
    run_test(tc, s);
}
