#include "software/simulated_tests/validation/terminating_function_validator.h"

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_that_does_nothing_reports_success)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {};

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_that_has_code_but_does_not_yield_reports_success)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        int foo = 0;
        int bar = 3;
        int baz = foo + bar;
        baz++;
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_that_yields_once_succeeds_on_the_second_execution)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield("Test message");
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("Test message", function_validator.currentErrorMessage());
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_that_yields_five_time_succeeds_on_the_sixth_execution)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield("Test message");
        yield("Test message");
        yield("Test message");
        yield("Test message");
        yield("Test message");
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);

    for (unsigned int i = 0; i < 5; i++)
    {
        bool result = function_validator.executeAndCheckForSuccess();
        EXPECT_FALSE(result);
        EXPECT_EQ("Test message", function_validator.currentErrorMessage());
    }

    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_with_early_return_reports_success_after_return)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield("First validation not done yet");
        return;
        yield("Second validation not done yet");
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);

    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("First validation not done yet", function_validator.currentErrorMessage());

    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_with_single_loop_succeeds_after_loop_termination)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->ball().position().x() < 0)
        {
            yield("The ball's x position is not less than 0");
        }
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);

    world->updateBall(
        Ball(BallState(Point(-1, 0), Vector(0, 0)), Timestamp::fromSeconds(0)));
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("The ball's x position is not less than 0",
              function_validator.currentErrorMessage());

    world->updateBall(
        Ball(BallState(Point(-0.1, 0), Vector(0, 0)), Timestamp::fromSeconds(0)));
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("The ball's x position is not less than 0",
              function_validator.currentErrorMessage());

    world->updateBall(
        Ball(BallState(Point(0.05, 0), Vector(0, 0)), Timestamp::fromSeconds(0)));
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest,
     test_validation_function_with_two_loops_succeeds_after_both_loops_terminate_in_order)
{
    // This validation function will only pass if the ball's x-coordinate becomes positive
    // before the ball's y-coordinate becomes positive
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->ball().position().x() < 0)
        {
            yield("The ball's x position is not less than 0");
        }

        while (world->ball().position().y() < 0)
        {
            yield("The ball's y position not less than 0");
        }
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);

    world->updateBall(
        Ball(BallState(Point(-1, -1), Vector(0, 0)), Timestamp::fromSeconds(0)));
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("The ball's x position is not less than 0",
              function_validator.currentErrorMessage());

    world->updateBall(
        Ball(BallState(Point(1, -1), Vector(0, 0)), Timestamp::fromSeconds(0)));
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    EXPECT_EQ("The ball's y position not less than 0",
              function_validator.currentErrorMessage());

    world->updateBall(
        Ball(BallState(Point(1, 1), Vector(0, 0)), Timestamp::fromSeconds(0)));
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(TerminatingFunctionValidatorTest, test_validation_function_error_message)
{
    // This shows an example of using GoogleTest statements within a validation function.
    // Just like regular unit tests, if the condition is not met the test will fail.
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->gameState().isStopped())
        {
            yield("Test message");
        }
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    TerminatingFunctionValidator function_validator(validation_function, world);

    world->updateRefereeCommand(RefereeCommand::STOP);
    world->updateBall(
        Ball(BallState(Point(0, 0), Vector(1, 1)), Timestamp::fromSeconds(0)));
    for (unsigned int i = 0; i < 10; i++)
    {
        bool result = function_validator.executeAndCheckForSuccess();
        EXPECT_FALSE(result);
        EXPECT_EQ("Test message", function_validator.currentErrorMessage());
    }

    for (unsigned int i = 0; i < World::REFEREE_COMMAND_BUFFER_SIZE; i++)
    {
        world->updateRefereeCommand(RefereeCommand::HALT);
    }

    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}
