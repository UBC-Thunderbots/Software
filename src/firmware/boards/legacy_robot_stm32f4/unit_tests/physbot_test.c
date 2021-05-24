#include "main/util/physbot.h"

#include "main/physics.h"
#include "test.h"

START_TEST(test_setup_bot)
{
    dr_data_t states;
    states.vx    = 2.0f;
    states.vy    = -3.0f;
    states.x     = 1.1f;
    states.y     = 0.3f;
    states.angle = P_PI;

    float destination[3] = {0.0f, 0.0f, 0.0f};
    float major_vec[2]   = {1.0f, 0.0f};
    float minor_vec[2]   = {0.0f, 1.0f};
    float dr[2]          = {destination[0] - states.x, destination[1] - states.y};
    float v[2]           = {states.vx, states.vy};

    PhysBot pb = setup_bot(states, destination, major_vec, minor_vec);

    ck_assert_float_eq_tol(destination[0] - states.x, pb.dr[0], TOL);
    ck_assert_float_eq_tol(destination[1] - states.y, pb.dr[1], TOL);

    ck_assert_float_eq_tol(major_vec[0], pb.major_vec[0], TOL);
    ck_assert_float_eq_tol(major_vec[1], pb.major_vec[1], TOL);
    ck_assert_float_eq_tol(minor_vec[0], pb.minor_vec[0], TOL);
    ck_assert_float_eq_tol(minor_vec[1], pb.minor_vec[1], TOL);

    ck_assert_float_eq_tol(P_PI, pb.rot.disp, TOL);

    ck_assert_float_eq_tol(shared_physics_dot2D(major_vec, dr), pb.maj.disp, TOL);
    ck_assert_float_eq_tol(shared_physics_dot2D(major_vec, v), pb.maj.vel, TOL);
    ck_assert_float_eq_tol(shared_physics_dot2D(minor_vec, dr), pb.min.disp, TOL);
    ck_assert_float_eq_tol(shared_physics_dot2D(minor_vec, v), pb.min.vel, TOL);

    ck_assert_float_eq_tol(0, pb.maj.accel, TOL);
    ck_assert_float_eq_tol(0, pb.maj.time, TOL);
    ck_assert_float_eq_tol(0, pb.min.accel, TOL);
    ck_assert_float_eq_tol(0, pb.min.time, TOL);
}
END_TEST

START_TEST(test_to_local_coords)
{
    float accel[3] = {0.0f, 0.0f, 0.0f};
    PhysBot pb;
    pb.maj.accel       = 1.5f;
    pb.min.accel       = 0.25f;
    float angle        = P_PI;
    float major_vec[2] = {1.0f, 0.0f};
    float minor_vec[2] = {0.0f, -1.0f};
    to_local_coords(accel, pb, angle, major_vec, minor_vec);
    ck_assert_float_eq_tol(-1.5f, accel[0], TOL);
    ck_assert_float_eq_tol(0.25f, accel[1], TOL);
}
END_TEST

void run_physbot_test(void)
{
    // Put the name of the suite of tests in here
    Suite *s = suite_create("PhysBot Test");
    // Creates a test case that you can add all of the tests to
    TCase *tc_core = tcase_create("Core");
    // add the tests for this file here
    tcase_add_test(tc_core, test_setup_bot);
    tcase_add_test(tc_core, test_to_local_coords);
    // run the tests
    run_test(tc_core, s);
}
