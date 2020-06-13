#include "software/simulated_tests/validation/continuous_function_validator.h"

#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/test_util/test_util.h"

TEST(ContinuousFunctionValidatorTest,
     test_validation_function_that_does_nothing_does_not_report_failure)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {};

    auto world           = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    world->mutableBall() = Ball(Point(-0.1, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    ContinuousFunctionValidator function_validator(validation_function, world);

    for (unsigned int i = 0; i < 10; i++)
    {
        function_validator.executeAndCheckForFailures();
    }
}

TEST(ContinuousFunctionValidatorTest,
     test_world_updated_correctly_between_validation_function_restarts)
{
    // This validation_functions uses exceptions as a way for the test to observe it's
    // internal state
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        if (world->ball().position().x() < -1)
        {
            throw std::runtime_error("x < -1");
        }
        else if (world->ball().position().x() < 0)
        {
            throw std::runtime_error("x < 0");
        }
        else if (world->ball().position().x() < 1)
        {
            throw std::runtime_error("x < 1");
        }
        return;
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    ContinuousFunctionValidator function_validator(validation_function, world);

    world->mutableBall() = Ball(Point(-2, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    try
    {
        function_validator.executeAndCheckForFailures();
        ADD_FAILURE() << "Expected exception type not thrown" << std::endl;
    }
    catch (std::runtime_error& e)
    {
        EXPECT_STREQ("x < -1", e.what());
    }

    // Check one more time without updating the world
    try
    {
        function_validator.executeAndCheckForFailures();
        ADD_FAILURE() << "Expected exception type not thrown" << std::endl;
    }
    catch (std::runtime_error& e)
    {
        EXPECT_STREQ("x < -1", e.what());
    }

    world->mutableBall() = Ball(Point(0.5, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    try
    {
        function_validator.executeAndCheckForFailures();
        ADD_FAILURE() << "Expected exception type not thrown" << std::endl;
    }
    catch (std::runtime_error& e)
    {
        EXPECT_STREQ("x < 1", e.what());
    }

    world->mutableBall() = Ball(Point(-0.5, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    try
    {
        function_validator.executeAndCheckForFailures();
        ADD_FAILURE() << "Expected exception type not thrown" << std::endl;
    }
    catch (std::runtime_error& e)
    {
        EXPECT_STREQ("x < 0", e.what());
    }
}

TEST(ContinuousFunctionValidatorTest,
     test_coroutines_yield_correctly_in_validation_function)
{
    // This validation_functions uses exceptions as a way for the test to observe it's
    // internal state The exception will not be reached until the 3rd function call
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield();
        yield();
        throw std::runtime_error("coroutine reached end of yield statements");
    };

    auto world = std::make_shared<World>(::TestUtil::createBlankTestingWorld());
    ContinuousFunctionValidator function_validator(validation_function, world);

    function_validator.executeAndCheckForFailures();
    function_validator.executeAndCheckForFailures();
    EXPECT_THROW(function_validator.executeAndCheckForFailures(), std::runtime_error);
}
