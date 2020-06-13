#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "software/logger/logger.h"
#include "software/simulated_tests/mock_ai_wrapper.h"
#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * This class replaces the AIWrapper from the SimulatedTest with a MockAIWrapper,
 * so that any AI logic does not affect these tests.
 *
 * These tests are designed to validate that all the components of the simulated testing
 * system work together as expected. This includes the SimulatorBackend simulating and
 * publishing the world correctly, the WorldStateValidator performing checks as expected,
 * and the flow of control going through the AI and then back to the SimulatedBackend. We
 * use the MockAIWrapper so that we are not testing any behaviour relating to the AI,
 * just the testing infrastructure.
 */
class MockSimulatedTest : public SimulatedTest
{
   protected:
    void SetUp() override
    {
        LoggerSingleton::initializeLogger();
        backend = std::make_shared<SimulatorBackend>(
            Duration::fromMilliseconds(5), Duration::fromSeconds(1.0 / 30.0),
            SimulatorBackend::SimulationSpeed::FAST_SIMULATION);
        world_state_validator = std::make_shared<WorldStateValidator>();
        mock_ai_wrapper       = std::make_shared<MockAIWrapper>();

        backend->Subject<World>::registerObserver(world_state_validator);
        world_state_validator->Subject<World>::registerObserver(mock_ai_wrapper);
        mock_ai_wrapper->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
    }

    std::shared_ptr<MockAIWrapper> mock_ai_wrapper;
};

// NOTE: All these tests use validation functions that assert various things about the
// timestamp of the world. There is no physics directly involved in these tests, they are
// as simple as possible and only test the behavior of the validation functions and
// test pipeline

TEST_F(MockSimulatedTest, test_single_validation_function_passes_before_timeout)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.5))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(1));
    EXPECT_TRUE(test_passed);
}

TEST_F(MockSimulatedTest, test_single_validation_function_fails_if_it_times_out)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(1.0))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_FALSE(test_passed);
}

TEST_F(MockSimulatedTest,
       test_gtest_expect_statement_in_validation_function_causes_test_to_fail)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.5))
            {
                yield();
            }

            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.4));
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);

    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
                                validation_functions, continous_validation_functions,
                                Duration::fromSeconds(1.0)),
                            "Timestamp");
}

TEST_F(MockSimulatedTest, test_multiple_validation_function_pass_before_timeout)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield();
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.6))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.7));
    EXPECT_TRUE(test_passed);
}

TEST_F(MockSimulatedTest, test_should_fail_if_not_all_validation_functions_pass)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.4))
            {
                yield();
            }
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(0.8))
            {
                yield();
            }
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.7));
    EXPECT_FALSE(test_passed);
}

TEST_F(MockSimulatedTest, test_single_continuous_validation_function_passes)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        }};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_TRUE(test_passed);
}

TEST_F(MockSimulatedTest, test_multiple_continuous_validation_function_passes)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(100));
        }};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_TRUE(test_passed);
}

TEST_F(MockSimulatedTest,
       test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail)
{
    World world = ::TestUtil::createBlankTestingWorld();

    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
    // to write this failing function in such a way that it will only fail once during the
    // test. To do this we check the timestamp very close to the test timeout
    auto failing_validation_function = [](std::shared_ptr<World> world_ptr,
                                          ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.5));
    };

    auto passing_validation_function = [](std::shared_ptr<World> world_ptr,
                                          ValidationCoroutine::push_type& yield) {
        EXPECT_GE(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0));
    };

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        passing_validation_function, failing_validation_function};

    backend->startSimulation(world);
    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
                                validation_functions, continous_validation_functions,
                                Duration::fromSeconds(0.5)),
                            "Timestamp");
}

// This test is basically the same as
// test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail but
// exists because there was a bug with the FunctionValidator and
// ContinuousFunctionValidator that caused only the last validation_function in the
// vectors to be run, so we re-order the validation_functions in this test to catch future
// failures of this type
TEST_F(
    MockSimulatedTest,
    test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail_with_different_test_order)
{
    World world = ::TestUtil::createBlankTestingWorld();

    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
    // to write this failing function in such a way that it will only fail once during the
    // test. To do this we check the timestamp very close to the test timeout
    auto failing_validation_function = [](std::shared_ptr<World> world_ptr,
                                          ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.5));
    };

    auto passing_validation_function = [](std::shared_ptr<World> world_ptr,
                                          ValidationCoroutine::push_type& yield) {
        EXPECT_GE(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0));
    };

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        failing_validation_function,
        passing_validation_function,
    };

    backend->startSimulation(world);
    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
                                validation_functions, continous_validation_functions,
                                Duration::fromSeconds(0.5)),
                            "Timestamp");
}

TEST_F(MockSimulatedTest,
       test_validation_and_continuous_validation_functions_pass_together)
{
    World world = ::TestUtil::createBlankTestingWorld();

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0.4));
        },
    };

    std::vector<ValidationFunction> continous_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_GE(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(0));
        },
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_LT(world_ptr->getMostRecentTimestamp(), Timestamp::fromSeconds(100));
        }};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_TRUE(test_passed);
}
