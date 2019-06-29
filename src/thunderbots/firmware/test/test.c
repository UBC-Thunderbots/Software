
#include "math_test.h"
#include "matrix_test.h"
#include "move_test.h"
#include "physbot_test.h"
#include "physics_test.h"
#include "quadratic_test.h"
#include "shoot_test.h"
#include "util_test.h"
#include <stdio.h>
#include <stdlib.h>
#include "check.h"
#include "test.h"
#include <stdio.h>

#include "check.h"
static int number_failed = 0;
#include "test.h"


void run_test(TCase *tc, Suite *s) {
static int number_failed = 0;
    suite_add_tcase(s, tc);

    SRunner *sr = srunner_create(s);
    srunner_run_all(sr, CK_NORMAL);
void run_test(TCase *tc, Suite *s) {
    number_failed += srunner_ntests_failed(sr);
    suite_add_tcase(s, tc);
    srunner_free(sr);
    SRunner *sr = srunner_create(s);
    printf("\n");
    srunner_run_all(sr, CK_NORMAL);
}
    number_failed += srunner_ntests_failed(sr);

/**
    srunner_free(sr);
 * Main entry point for the test cases. Each test to run should 
    printf("\n");
 * be wrapped inside a function that should be added here so that
}
 * when the main function is called each test is run.

 */
/**
int main(void)
 * Main entry point for the test cases. Each test to run should 
{
    printf("\nStart Tests\n");
 * be wrapped inside a function that should be added here so that
    run_math_test();
    run_matrix_test();
    run_move_test();
    run_physbot_test();
    run_physics_test();
    run_quadratic_test();
    run_shoot_test();
    run_util_test();
 * when the main function is called each test is run.
    (number_failed == 0) ? printf("All tests passed.\n") : printf("%d Tests failed.\n\n", number_failed);
 */
    return 0;
int main(void)
{
}
    printf("\nStart Tests\n");
    run_math_test();
    run_matrix_test();
    run_move_test();
    run_physbot_test();
    run_physics_test();
    run_quadratic_test();
    run_shoot_test();
    run_util_test();
    (number_failed == 0) ? printf("All tests passed.\n") : printf("%d Tests failed.\n\n", number_failed);
    return 0;
}
