#include <math.h>

#include "check.h"
#include "test.h"

START_TEST(test_atan2f_first_quadrant)
{
    ck_assert_float_eq(M_PI / 4.0f, atan2f(1, 1));
}
END_TEST

START_TEST(test_atan2f_second_quadrant)
{
    ck_assert_float_eq(3 * M_PI / 4.0f, atan2f(1, -1));
}
END_TEST

START_TEST(test_atan2f_third_quadrant)
{
    ck_assert_float_eq(-3 * M_PI / 4.0f, atan2f(-1, -1));
}
END_TEST

START_TEST(test_atan2f_fourth_quadrant)
{
    ck_assert_float_eq(-M_PI / 4.0f, atan2f(-1, 1));
}
END_TEST

START_TEST(test_atan2f_positive_x_axis)
{
    ck_assert_float_eq(0, atan2f(0, 1));
}
END_TEST

START_TEST(test_atan2f_positive_y_axis)
{
    ck_assert_float_eq(M_PI / 2.0f, atan2f(1, 0));
}
END_TEST

START_TEST(test_atan2f_negative_x_axis)
{
    ck_assert_float_eq(M_PI, atan2f(0, -1));
}
END_TEST

START_TEST(test_atan2f_negative_y_axis)
{
    ck_assert_float_eq(-M_PI / 2.0f, atan2f(-1, 0));
}
END_TEST

void run_math_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("Math Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc = tcase_create("Core");
    // add the tests for this file here
    tcase_add_test(tc, test_atan2f_first_quadrant);
    tcase_add_test(tc, test_atan2f_second_quadrant);
    tcase_add_test(tc, test_atan2f_third_quadrant);
    tcase_add_test(tc, test_atan2f_fourth_quadrant);
    tcase_add_test(tc, test_atan2f_positive_x_axis);
    tcase_add_test(tc, test_atan2f_positive_y_axis);
    tcase_add_test(tc, test_atan2f_negative_x_axis);
    tcase_add_test(tc, test_atan2f_negative_y_axis);
    // run the tests
    run_test(tc, s);
}
