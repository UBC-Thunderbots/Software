#include "main/util/util.h"

#include "check.h"
#include "test.h"


START_TEST(test_fmax_of_array)
{
    float test_array[6] = {1.0f, 6.0f, 3.0f, -100.0f, 4.1f, 0.0f};
    float result        = fmax_of_array(test_array, 6);
    ck_assert_float_eq_tol(6.0f, result, TOL);
}
END_TEST

START_TEST(test_fmin_of_array)
{
    float test_array[6] = {-100.03f, -40.01f, -1000.0f, 45.1f, 23.3f, 0.0f};
    float result        = fmin_of_array(test_array, 6);
    ck_assert_float_eq_tol(-1000.0f, result, TOL);
}
END_TEST

START_TEST(test_argmax)
{
    float test_array[5] = {-10.0f, 3.0f, -12.0f, 45.0f, 4.0f};
    unsigned result     = argmax(test_array, 5);
    ck_assert_int_eq(3, result);
}
END_TEST

START_TEST(test_argmin)
{
    float test_array[5] = {-10.0f, 3.0f, -12.0f, 45.0f, 4.0f};
    unsigned result     = argmin(test_array, 5);
    ck_assert_int_eq(2, result);
}
END_TEST

START_TEST(test_fabs_of_array)
{
    float test_array[7] = {-3.0f, -2.1f, -1.2f, 0.0f, 1.1f, 2.5f, 3.6f};
    float abs_array[7];
    fabs_of_array(test_array, abs_array, 7);
    float expected_result[7] = {3.0f, 2.1f, 1.2f, 0.0f, 1.1f, 2.5f, 3.6f};
    unsigned i;
    for (i = 0; i < 7; i++)
    {
        ck_assert_float_eq_tol(expected_result[i], abs_array[i], TOL);
    }
}
END_TEST

START_TEST(test_limit_positive_above)
{
    float value = 10.0f;
    limit(&value, 5.0f);
    ck_assert_float_eq_tol(5.0f, value, TOL);
}
END_TEST

START_TEST(test_limit_positive_below)
{
    float value = 10.0f;
    limit(&value, 15.0f);
    ck_assert_float_eq_tol(10.0f, value, TOL);
}
END_TEST

START_TEST(test_limit_negative_above)
{
    float value = -5.0f;
    limit(&value, 10.0f);
    ck_assert_float_eq_tol(-5.0f, value, TOL);
}
END_TEST

START_TEST(test_limit_negative_below)
{
    float value = -10.0f;
    limit(&value, 5.0f);
    ck_assert_float_eq_tol(-5.0f, value, TOL);
}
END_TEST

START_TEST(test_radians)
{
    float degrees = 45.0f;
    ck_assert_float_eq_tol(M_PI / 4.0f, radians(degrees), TOL);
}
END_TEST

void run_util_test(void)
{
    Suite *s  = suite_create("Util Test");
    TCase *tc = tcase_create("Core");
    tcase_add_test(tc, test_fmax_of_array);
    tcase_add_test(tc, test_fmin_of_array);
    tcase_add_test(tc, test_argmax);
    tcase_add_test(tc, test_argmin);
    tcase_add_test(tc, test_fabs_of_array);
    tcase_add_test(tc, test_limit_positive_above);
    tcase_add_test(tc, test_limit_positive_below);
    tcase_add_test(tc, test_limit_negative_above);
    tcase_add_test(tc, test_limit_negative_below);
    tcase_add_test(tc, test_radians);
    run_test(tc, s);
}
