#include "software/simulated_tests/validation/non_terminating_function_validator.h"

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"

TEST(NonTerminatingFunctionValidatorTest,
     test_validation_function_that_does_nothing_does_not_report_failure)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {};

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    world->updateBall(
        Ball(BallState(Point(-0.1, 0), Vector(0, 0)), Timestamp::fromSeconds(0)));
    NonTerminatingFunctionValidator function_validator(validation_function, world);

    for (unsigned int i = 0; i < 10; i++)
    {
        EXPECT_FALSE(function_validator.executeAndCheckForFailures());
    }
}

TEST(NonTerminatingFunctionValidatorTest, test_yielding_error_message)
{
    // This validation_functions uses exceptions as a way for the test to observe it's
    // internal state The exception will not be reached until the 3rd function call
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield("This is an error message 1");
        yield("This is an error message 2");
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    NonTerminatingFunctionValidator function_validator(validation_function, world);
    EXPECT_EQ(function_validator.executeAndCheckForFailures(),
              "This is an error message 1");
    EXPECT_EQ(function_validator.executeAndCheckForFailures(),
              "This is an error message 2");
}

TEST(NonTerminatingFunctionValidatorTest,
     test_world_updated_correctly_between_validation_function_restarts)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        if (world->ball().position() != Point(1, 1))
        {
            yield("Ball not at (1,1)");
        }
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    NonTerminatingFunctionValidator function_validator(validation_function, world);
    EXPECT_EQ(function_validator.executeAndCheckForFailures(), "Ball not at (1,1)");
    world->updateBall(
        Ball(BallState(Point(1, 1), Vector()), Timestamp::fromSeconds(123)));
    EXPECT_FALSE(function_validator.executeAndCheckForFailures());
}
