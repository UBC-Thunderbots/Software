#include "software/simulated_tests/simulated_play_test_fixture.h"

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
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
   protected:
    Field field = Field::createSSLDivisionBField();
    std::vector<RobotStateWithId> friendly_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(-4, 0)});
    std::vector<RobotStateWithId> enemy_robots =
        TestUtil::createStationaryRobotStatesWithId({Point(4, 0)});
};

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_validation_function_passes_before_timeout)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.5))
            {
                yield("Waiting for timestamp of at least 0.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(1.0));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_validation_function_fails_if_it_times_out)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1.0))
            {
                yield("Waiting for timestamp of at least 1s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    EXPECT_NONFATAL_FAILURE(
        runTest(field, ball_state, friendly_robots, enemy_robots,
                terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.5)),
        "Waiting for timestamp of at least 1s");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_multiple_validation_function_pass_before_timeout)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield("Waiting for timestamp of at least 0.4s");
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.6))
            {
                yield("Waiting for timestamp of at least 0.6s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.7));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_should_fail_if_not_all_validation_functions_pass)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield("Waiting for timestamp of at least 0.4s");
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.8))
            {
                yield("Waiting for timestamp of at least 0.8s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {};

    EXPECT_NONFATAL_FAILURE(
        runTest(field, ball_state, friendly_robots, enemy_robots,
                terminating_validation_functions, non_terminating_validation_functions,
                Duration::fromSeconds(0.6)),
        "Waiting for timestamp of at least 0.8s");
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_single_non_terminating_validation_function_passes)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0))
            {
                yield("Timestamp was less than 0");
            }
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_multiple_non_terminating_validation_function_passes)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0))
            {
                yield("Timestamp was less than 0");
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() >= Timestamp::fromSeconds(100))
            {
                yield("Timestamp was greater than or equal to 100");
            }
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}

TEST_F(SimulatedPlayTestFixtureTest,
       test_terminating_and_non_terminating_validation_functions_pass_together)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield("Waiting for timestamp of at least 0.4s");
            }
        },
    };

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.0))
            {
                yield("Timestamp was less than 0");
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() > Timestamp::fromSeconds(100.0))
            {
                yield("Timestamp was greater than 100");
            }
        }};

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(0.5));
}

TEST_F(SimulatedPlayTestFixtureTest,
       non_terminating_validation_functions_are_ignored_after_terminating_functions_pass)
{
    BallState ball_state(Point(0, 0), Vector(0, 0));

    std::vector<ValidationFunction> terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.5))
            {
                yield("Waiting for timestamp of at least 0.5s");
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0))
            {
                yield("Timestamp was less than 0");
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            if (world_ptr->getMostRecentTimestamp() >= Timestamp::fromSeconds(1.0))
            {
                yield("Timestamp was greater than or equal to 1.0");
            }
        }};


    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(1.5));
}
