#include "main/primitives/move.h"

#include "check.h"
#include "main/physics.h"
#include "main/util/physbot.h"
#include "shared/robot_constants.h"
#include "shared/util.h"
#include "test.h"

// Common PhysBot to use throughout tests
static PhysBot pb;

START_TEST(test_plan_move_rotation_large)
{
    // expected values were determined from manually
    // doing the math that plan_move_rotation does
    pb.rot.disp = 20.0f * P_PI / 180.0f;
    plan_move_rotation(&pb, 3.0f);
    ck_assert_double_eq(TIME_HORIZON, pb.rot.time);
    ck_assert_double_eq_tol(6.98132, pb.rot.vel, 0.001);
    ck_assert_double_eq_tol(MAX_T_A, pb.rot.accel, 0.001);
}
END_TEST

START_TEST(test_plan_move_rotation_small)
{
    // expected values were determined from manually
    // doing the math that plan_move_rotation does
    pb.rot.disp = 2.0f * P_PI / 180.0f;
    plan_move_rotation(&pb, 2.0f);
    ck_assert_double_eq(TIME_HORIZON, pb.rot.time);
    ck_assert_double_eq_tol(0.698132, pb.rot.vel, 0.001);
    ck_assert_double_eq_tol(-26.0374, pb.rot.accel, 0.001);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_0)
{
    // moving along x-axis, facing forward and same final angle
    unsigned index = choose_wheel_axis(1, 0, 0, 0);
    ck_assert_int_eq(0, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_1)
{
    // moving along x-axis, facing forward, final angle
    // is almost opposite to direction of movement
    unsigned index = choose_wheel_axis(1, 0, 0, radians(-170.0f));
    ck_assert_int_eq(1, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_2)
{
    // moving along x-axis, facing forward, final angle
    // is almost opposite to direction of movement
    unsigned index = choose_wheel_axis(1, 0, 0, radians(170.0f));
    ck_assert_int_eq(2, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_3)
{
    // moving along x-axis, facing forward
    // final angle chooses wheel index 3
    unsigned index = choose_wheel_axis(1, 0, 0, radians(-20.0f));
    ck_assert_int_eq(3, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_4)
{
    // moving along x-axis, facing forward
    // final angle chooses wheel index 4
    unsigned index = choose_wheel_axis(1, 0, 0, radians(-50.0f));
    ck_assert_int_eq(4, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_5)
{
    // moving along x-axis, facing forward
    // final angle chooses wheel index 5
    unsigned index = choose_wheel_axis(1, 0, 0, P_PI / 2.0f);
    ck_assert_int_eq(5, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_6)
{
    // moving along x-axis, facing forward
    // final angle chooses wheel index 6
    unsigned index = choose_wheel_axis(1, 0, 0, radians(50.0f));
    ck_assert_int_eq(6, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_7)
{
    // moving along x-axis, facing forward
    // final angle chooses wheel index 7
    unsigned index = choose_wheel_axis(1, 0, 0, radians(-120.0f));
    ck_assert_int_eq(7, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_8)
{
    // moving along x-axis, facing 30 degrees
    // final angle is perpendicular to initial angle
    unsigned index = choose_wheel_axis(1, 0, radians(30.0f), radians(-120.0f));
    ck_assert_int_eq(7, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_9)
{
    // moving along negative x-axis, facing 30 degrees
    // final angle is perpendicular to initial angle
    unsigned index = choose_wheel_axis(-1, 0, radians(30.0f), radians(-120.0f));
    ck_assert_int_eq(6, index);
}
END_TEST

START_TEST(test_choose_wheel_axis_case_10)
{
    // moving along the vector <2, 1>, facing 30 degrees, final angle is
    // roughly perpendicular to direction of movement
    unsigned index = choose_wheel_axis(2, 1, radians(30.0f), radians(140.0f));
    ck_assert_int_eq(5, index);
}
END_TEST

/**
 * Setup function for the move tests. Initializes some parameters
 * for the PhysBot we are going to use.
 */
void move_test_init(void)
{
    pb.maj.disp = APPROACH_LIMIT + 1;
    pb.dr[0]    = 0;
    pb.dr[1]    = 0;
    pb.rot.disp = 0;
    pb.maj.time = 0.0f;
}

/**
 * Test function manager for move.c primitive
 *
 * In many of the test cases for the move primitive the setup
 * is rigged to choose a specific wheel axis. This was determined
 * via analyzing the layout of the robot and the possible choices
 * it could have for a wheel axis when moving in a certin direction.
 */
void run_move_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("Move Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc_core = tcase_create("Core");
    // add a setup function
    tcase_add_checked_fixture(tc_core, move_test_init, NULL);
    // add the tests for this file here
    tcase_add_test(tc_core, test_plan_move_rotation_large);
    tcase_add_test(tc_core, test_plan_move_rotation_small);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_0);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_1);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_2);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_3);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_4);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_5);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_6);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_7);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_8);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_9);
    tcase_add_test(tc_core, test_choose_wheel_axis_case_10);
    // run the tests
    run_test(tc_core, s);
}
