#include <check.h>

// Tolerance to pass to ck_assert_float_eq
#define TOL 0.00001f


/**
 * Generic function to call that will run a Suite of test cases.
 * Once a TCase has had a bunch of tests added to it, pass it to
 * this function along with a Suite to run the tests.
 *
 * @param tc Should have a series of test cases added to it
 * @param s The Suite should have been created with an appropriate name
 * for the test that it manages
 * @return void
 */
void run_test(TCase *tc, Suite *s);
