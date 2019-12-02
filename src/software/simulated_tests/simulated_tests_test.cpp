#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include "software/simulated_tests/mock_ai_wrapper.h"
#include "software/simulated_tests/simulated_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"
#include "software/util/logger/init.h"
#include "software/util/time/duration.h"
#include "software/util/time/timestamp.h"
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
        Util::Logger::LoggerSingleton::initializeLogger();
        backend = std::make_shared<SimulatorBackend>(
            Duration::fromMilliseconds(5), Duration::fromSeconds(1.0 / 30.0),
            SimulatorBackend::SimulationSpeed::FAST_SIMULATION);
        world_state_validator = std::make_shared<WorldStateValidator>();
        mock_ai_wrapper       = std::make_shared<MockAIWrapper>();

        backend->Subject<World>::registerObserver(world_state_validator);
        world_state_validator->Subject<World>::registerObserver(mock_ai_wrapper);
        mock_ai_wrapper->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);
//        std::cout << "setup done" << std::endl;
    }

    void TearDown() override
    {
        backend->stopSimulation();
    }

    std::shared_ptr<MockAIWrapper> mock_ai_wrapper;
};

//TEST_F(MockSimulatedTest, test_single_validation_function_passes_before_timeout)
//{
//    World world         = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));
//
//    std::vector<ValidationFunction> validation_functions = {
//        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
//            while (world_ptr->ball().position().x() < 3)
//            {
//                yield();
//            }
//        }};
//
//    std::vector<ValidationFunction> continous_validation_functions = {};
//
//    backend->startSimulation(world);
//    bool test_passed = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(1));
//    EXPECT_TRUE(test_passed);
//}

//TEST_F(MockSimulatedTest, test_single_validation_function_fails_if_it_times_out)
//{
//    World world         = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));
//
//    std::vector<ValidationFunction> validation_functions = {
//        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
//            while (world_ptr->ball().position().x() < 3)
//            {
//                yield();
//            }
//        }};
//
//    std::vector<ValidationFunction> continous_validation_functions = {};
//
//    backend->startSimulation(world);
//    bool test_passed = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
//    EXPECT_FALSE(test_passed);
//}

TEST_F(MockSimulatedTest,
       test_gtest_expect_statement_in_validation_function_causes_test_to_fail)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(0, 0), Vector(4, 1.5), Timestamp::fromSeconds(0));

    std::vector<ValidationFunction> validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            while (world_ptr->ball().position().x() < 3)
            {
                yield();
            }

            EXPECT_LT(world_ptr->ball().position().x(), 2.0);
        }};

    std::vector<ValidationFunction> continous_validation_functions = {};

    backend->startSimulation(world);

    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
                                validation_functions, continous_validation_functions,
                                Duration::fromSeconds(1.0)),
                            "position");
}

//TEST_F(MockSimulatedTest, test_multiple_validation_function_pass_before_timeout)
//{
//    World world         = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() = Ball(Point(-1, -1), Vector(3, 2), Timestamp::fromSeconds(0));
//
//    auto ball_crosses_x_axis = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        while (world_ptr->ball().position().x() < 0)
//        {
//            yield();
//        }
//    };
//
//    auto ball_crosses_y_axis = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        while (world_ptr->ball().position().y() < 0)
//        {
//            yield();
//        }
//    };
//
//    std::vector<ValidationFunction> validation_functions = {ball_crosses_x_axis,
//                                                            ball_crosses_y_axis};
//
//    std::vector<ValidationFunction> continous_validation_functions = {};
//
//    backend->startSimulation(world);
//    bool test_passed = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(1.0));
//    EXPECT_TRUE(test_passed);
//}

//TEST_F(MockSimulatedTest, test_should_fail_if_not_all_validation_functions_pass)
//{
//    World world         = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() = Ball(Point(-1, -1), Vector(3, 0.2), Timestamp::fromSeconds(0));
//
//    auto ball_crosses_x_axis = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        while (world_ptr->ball().position().x() < 0)
//        {
//            yield();
//        }
//    };
//
//    auto ball_crosses_y_axis = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        while (world_ptr->ball().position().y() < 0)
//        {
//            yield();
//        }
//    };
//
//    std::vector<ValidationFunction> validation_functions = {ball_crosses_x_axis,
//                                                            ball_crosses_y_axis};
//
//    std::vector<ValidationFunction> continous_validation_functions = {};
//
//    backend->startSimulation(world);
//    bool test_passed = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(1.0));
//    EXPECT_FALSE(test_passed);
//}

TEST_F(MockSimulatedTest, test_single_continuous_validation_function_passes)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() =
        Ball(Point(-1, -1), Vector(0.5, 0.2), Timestamp::fromSeconds(0));

    auto ball_velocity_small = [](std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().velocity().length(), 1.0);
    };

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        ball_velocity_small};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_TRUE(test_passed);
}

TEST_F(MockSimulatedTest, test_multiple_continuous_validation_function_passes)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() =
        Ball(Point(0.5, 0.5), Vector(0.5, 0.2), Timestamp::fromSeconds(0));

    auto ball_velocity_small = [](std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().velocity().length(), 1.0);
    };

    auto ball_in_positive_x_positive_y_quadrant =
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            EXPECT_GT(world_ptr->ball().position().x(), 0.0);
            EXPECT_GT(world_ptr->ball().position().y(), 0.0);
        };

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
        ball_velocity_small, ball_in_positive_x_positive_y_quadrant};

    backend->startSimulation(world);
    bool test_passed = world_state_validator->waitForValidationToPass(
        validation_functions, continous_validation_functions, Duration::fromSeconds(0.5));
    EXPECT_TRUE(test_passed);
}

//TEST_F(MockSimulatedTest,
//       test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail)
//{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() =
//        Ball(Point(0.5, 0.5), Vector(0.5, 0.2), Timestamp::fromSeconds(0));
//
//    auto ball_velocity_small = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        EXPECT_LT(world_ptr->ball().velocity().length(), 1.0);
//    };
//
//    auto ball_in_positive_x_positive_y_quadrant =
//        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
//            EXPECT_GT(world_ptr->ball().position().x(), 0.0);
//            EXPECT_GT(world_ptr->ball().position().y(), 0.0);
//        };
//
//    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
//    // to write this failing function in such a way that it will only fail once during the
//    // test. To do this we check the timestamp very close to the test timeout
//    auto failing_validation_function = [](std::shared_ptr<World> world_ptr,
//                                          ValidationCoroutine::push_type& yield) {
//        EXPECT_LT(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.5));
//    };
//
//    std::vector<ValidationFunction> validation_functions = {};
//
//    std::vector<ValidationFunction> continous_validation_functions = {
//        ball_velocity_small, ball_in_positive_x_positive_y_quadrant,
//        failing_validation_function};
//
//    backend->startSimulation(world);
//    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
//                                validation_functions, continous_validation_functions,
//                                Duration::fromSeconds(0.5)),
//                            "Timestamp");
//}

// This test is basically the same as test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail
// but exists because there was a bug with the FunctionValidator and ContinuousFunctionValidator that caused only
// the last validation_function in the vectors to be run, so we re-order the validation_functions in this
// test to catch future failures of this type
TEST_F(MockSimulatedTest,
       test_gtest_expect_statement_in_continuous_validation_function_causes_test_to_fail_with_different_test_order)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() =
            Ball(Point(0.5, 0.5), Vector(0.5, 0.2), Timestamp::fromSeconds(0));

    auto ball_velocity_small = [](std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().velocity().length(), 1.0);
    };

    auto ball_in_positive_x_positive_y_quadrant =
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                EXPECT_GT(world_ptr->ball().position().x(), 0.0);
                EXPECT_GT(world_ptr->ball().position().y(), 0.0);
            };

    // Because the EXPECT_NONFATAL_FAILURE macro only captures a single failure, we have
    // to write this failing function in such a way that it will only fail once during the
    // test. To do this we check the timestamp very close to the test timeout
    auto failing_validation_function = [](std::shared_ptr<World> world_ptr,
                                          ValidationCoroutine::push_type& yield) {
        EXPECT_LT(world_ptr->ball().lastUpdateTimestamp(), Timestamp::fromSeconds(0.5));
    };

    std::vector<ValidationFunction> validation_functions = {};

    std::vector<ValidationFunction> continous_validation_functions = {
            failing_validation_function,
            ball_velocity_small, ball_in_positive_x_positive_y_quadrant};

    backend->startSimulation(world);
    EXPECT_NONFATAL_FAILURE(world_state_validator->waitForValidationToPass(
            validation_functions, continous_validation_functions,
            Duration::fromSeconds(0.5)),
                            "Timestamp");
}

//TEST_F(MockSimulatedTest,
//       test_validation_and_continuous_validation_functions_pass_together)
//{
//    World world = ::Test::TestUtil::createBlankTestingWorld();
//    world.mutableBall() =
//        Ball(Point(0.5, 0.5), Vector(2.0, 0.2), Timestamp::fromSeconds(0));
//
//    auto ball_velocity_small = [](std::shared_ptr<World> world_ptr,
//                                  ValidationCoroutine::push_type& yield) {
//        EXPECT_LT(world_ptr->ball().velocity().length(), 2.5);
//    };
//
//    auto ball_in_positive_x_positive_y_quadrant =
//        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
//            EXPECT_GT(world_ptr->ball().position().x(), 0.0);
//            EXPECT_GT(world_ptr->ball().position().y(), 0.0);
//        };
//
//    std::vector<ValidationFunction> validation_functions = {
//        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
//            while (world_ptr->ball().position().x() < 3)
//            {
//                yield();
//            }
//        }};
//
//    std::vector<ValidationFunction> continous_validation_functions = {
//        ball_velocity_small, ball_in_positive_x_positive_y_quadrant};
//
//    backend->startSimulation(world);
//    bool tests_pass = world_state_validator->waitForValidationToPass(
//        validation_functions, continous_validation_functions, Duration::fromSeconds(2.0));
//    EXPECT_TRUE(tests_pass);
//}
