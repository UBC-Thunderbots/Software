#include "main/physics.h"

#include "check.h"
#include "test.h"


START_TEST(test_shared_physics_dot2D)
{
    float a1[2] = {1.1f, 1.35f};
    float a2[2] = {2.6f, 5.1f};
    ck_assert_float_eq(9.745f, shared_physics_dot2D(a1, a2));
}
END_TEST

START_TEST(test_dot3D)
{
    float a1[3] = {1, 1, 1};
    float a2[3] = {2, 2, 2};
    ck_assert_float_eq(6, dot3D(a1, a2));
}
END_TEST

START_TEST(test_shared_physics_dotProduct)
{
    float a1[5] = {1, 1, 1, 1, 2};
    float a2[5] = {1, 2, 3, 4, 5};
    ck_assert_float_eq(20, shared_physics_dotProduct(a1, a2, 5));
}
END_TEST

START_TEST(test_shared_physics_minAngleDelta_lt_90)
{
    ck_assert_float_eq(30 * P_PI / 180.0f,
                       shared_physics_minAngleDelta(0, 30 * P_PI / 180.0f));
}
END_TEST

START_TEST(test_shared_physics_minAngleDelta_gt_90)
{
    ck_assert_float_eq(117 * P_PI / 180.0f,
                       shared_physics_minAngleDelta(0, 117 * P_PI / 180.0f));
}
END_TEST

START_TEST(test_shared_physics_minAngleDelta_both_non_zero)
{
    ck_assert_float_eq_tol(
        -47 * P_PI / 180.0f,
        shared_physics_minAngleDelta(15 * P_PI / 180.0f, -32 * P_PI / 180.0f), TOL);
}
END_TEST

START_TEST(test_shared_physics_minAngleDelta_big_angles)
{
    ck_assert_float_eq_tol(-173 * P_PI / 180.0f,
                           shared_physics_minAngleDelta(4 * P_PI, 187 * P_PI / 180.0f),
                           TOL);
}
END_TEST

/**
 * Test function manager for physics.c
 */
void run_physics_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("Physics Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc_core = tcase_create("Core");
    // add the tests for this file here
    tcase_add_test(tc_core, test_shared_physics_dot2D);
    tcase_add_test(tc_core, test_dot3D);
    tcase_add_test(tc_core, test_shared_physics_dotProduct);
    tcase_add_test(tc_core, test_shared_physics_minAngleDelta_lt_90);
    tcase_add_test(tc_core, test_shared_physics_minAngleDelta_gt_90);
    tcase_add_test(tc_core, test_shared_physics_minAngleDelta_both_non_zero);
    tcase_add_test(tc_core, test_shared_physics_minAngleDelta_big_angles);
    // run the tests
    run_test(tc_core, s);
}
