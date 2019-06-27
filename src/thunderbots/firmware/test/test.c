
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
#include <stdio.h>
#include "test.h"
#include "check.h"

#include "test.h"
static int number_failed = 0;


static int number_failed = 0;
void run_test(TCase *tc, Suite *s) {

void run_test(TCase *tc, Suite *s) {
    suite_add_tcase(s, tc);
    SRunner *sr = srunner_create(s);
    srunner_run_all(sr, CK_NORMAL);
    suite_add_tcase(s, tc);
    number_failed += srunner_ntests_failed(sr);
    SRunner *sr = srunner_create(s);
    srunner_run_all(sr, CK_NORMAL);
    srunner_free(sr);
    number_failed += srunner_ntests_failed(sr);
    printf("\n");
    srunner_free(sr);
}
    printf("\n");

/**
}
 * Main entry point for the test cases. Each test to run should 

 * be wrapped inside a function that should be added here so that
/**
 * when the main function is called each test is run.
 * Main entry point for the test cases. Each test to run should 
 */
 * be wrapped inside a function that should be added here so that
int main(void)
 * when the main function is called each test is run.
{
 */
    printf("\nStart Tests\n");
int main(void)
    run_math_test();
    run_matrix_test();
    run_move_test();
    run_physbot_test();
    run_physics_test();
    run_quadratic_test();
    run_shoot_test();
    run_util_test();
{
    (number_failed == 0) ? printf("All tests passed.\n") : printf("%d Tests failed.\n\n", number_failed);
    printf("\nStart Tests\n");
    return 0;
    run_math_test();
    run_matrix_test();
    run_move_test();
    run_physbot_test();
    run_physics_test();
    run_quadratic_test();
    run_shoot_test();
    run_util_test();
}
    (number_failed == 0) ? printf("All tests passed.\n") : printf("%d Tests failed.\n\n", number_failed);
    return 0;
}
