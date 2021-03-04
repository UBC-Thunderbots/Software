#include "software/simulated_tests/simulated_play_test_fixture.h"

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * These tests are designed to validate that all the components of the simulated testing
 * system work together as expected. This includes
 * - The simulation updating SensorFusion and the AI as expected
 * - The validation functions correctly validating the state of the
 *   world and failing when expected.
 *
 * NOTE: All these tests use validation functions that assert various things about the
 * timestamp of the world. There is no physics or AI directly involved in these tests,
 * they are as simple as possible and only test the behavior of the validation
 * functions and test pipeline
 */
class SimulatedPlayTestFixtureTest : public SimulatedPlayTestFixture
{
};

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_validation_function_passes_before_timeout)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.5))
            {
                yield("");
            }
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(1.0));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_validation_function_fails_if_it_times_out)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1.0))
            {
                yield("");
            }
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    EXPECT_NONFATAL_FAILURE(
        runTest(terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.5)),
        "timeout duration");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_gtest_expect_statement_in_validation_function_causes_test_to_fail)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() <= Timestamp::fromSeconds(0.5))
            {
                yield("");
            }

            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.5));
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    EXPECT_NONFATAL_FAILURE(
        runTest(terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(1.0)),
        "Timestamp");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_multiple_validation_function_pass_before_timeout)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield("");
            }
        },
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.6))
            {
                yield("");
            }
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.7));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_should_fail_if_not_all_validation_functions_pass)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield("");
            }
        },
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.8))
            {
                yield("");
            }
        }};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions =
        {};

    EXPECT_NONFATAL_FAILURE(
        runTest(terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.6)),
        "timeout duration");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_non_terminating_validation_function_passes)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_multiple_non_terminating_validation_function_passes)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        },
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(100));
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}

TEST_F(
    SimulatedPlayTestFixtureTest,
    test_failing_gtest_expect_statement_in_non_terminating_validation_function_causes_test_to_fail)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
    // to write this failing function in such a way that it will only fail once during the
    // test. To do this we check the timestamp very close to the test timeout
    auto failing_validation_function =
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.5));
        };

    auto passing_validation_function =
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        };

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions = {
        passing_validation_function, failing_validation_function};

    EXPECT_NONFATAL_FAILURE(
        runTest(terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.5)),
        "Timestamp");
}

TEST_F(
    SimulatedPlayTestFixtureTest,
    test_failing_gtest_expect_statement_in_non_terminating_validation_function_causes_test_to_fail_with_different_test_order)
{
    // This test is basically the same as
    // "test_failing_gtest_expect_statement_in_non_terminating_validation_function_causes_test_to_fail"
    // but exists as a regression test because there was a bug with the Terminating
    // and non-terminating FunctionValidator that caused only the last validation_function
    // in the vectors to be run, so we re-order the validation_functions in this test to
    // catch future failures of this type
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
    // to write this failing function in such a way that it will only fail once during the
    // test. To do this we check the timestamp very close to the test timeout
    auto failing_validation_function =
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.5));
        };

    auto passing_validation_function =
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        };

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {};

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions = {
        failing_validation_function,
        passing_validation_function,
    };

    EXPECT_NONFATAL_FAILURE(
        runTest(terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.5)),
        "Timestamp");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_terminating_and_non_terminating_validation_functions_pass_together)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));

    std::vector<TerminatingValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           TerminatingValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.4));
        },
    };

    std::vector<NonTerminatingValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        },
        [](std::shared_ptr<World> world_ptr,
           NonTerminatingValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(100));
        }};

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}
