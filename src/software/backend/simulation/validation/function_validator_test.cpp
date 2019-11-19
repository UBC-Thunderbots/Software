#include "software/backend/simulation/validation/function_validator.h"

#include <gtest/gtest.h>

#include <boost/coroutine2/all.hpp>

#include "software/backend/simulation/validation/validation_function.h"
#include "software/test_util/test_util.h"

TEST(FunctionValidatorTest, test_validation_function_that_does_nothing_reports_success)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {};

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_that_has_code_but_does_not_yield_reports_success)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        int foo = 0;
        int bar = 3;
        int baz = foo + bar;
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_that_yields_once_succeeds_on_the_second_execution)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield();
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);
    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);
    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_that_yields_five_time_succeeds_on_the_sixth_execution)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield();
        yield();
        yield();
        yield();
        yield();
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);

    for (unsigned int i = 0; i < 5; i++)
    {
        bool result = function_validator.executeAndCheckForSuccess();
        EXPECT_FALSE(result);
    }

    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_with_early_return_reports_success_after_return)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        yield();
        return;
        yield();
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);

    bool result = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    result = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_with_single_loop_succeeds_after_loop_termination)
{
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->ball().position().x() < 0)
        {
            yield();
        }
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);

    world->mutableBall() = Ball(Point(-1, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    bool result          = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    world->mutableBall() = Ball(Point(-0.1, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    world->mutableBall() = Ball(Point(0.05, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(FunctionValidatorTest,
     test_validation_function_with_two_loops_succeeds_after_both_loops_terminate_in_order)
{
    // This validation function will only pass if the ball's x-coordinate becomes positive
    // before the ball's y-coordinate becomes positive
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->ball().position().x() < 0)
        {
            yield();
        }

        while (world->ball().position().y() < 0)
        {
            yield();
        }
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);

    world->mutableBall() = Ball(Point(-1, -1), Vector(0, 0), Timestamp::fromSeconds(0));
    bool result          = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    world->mutableBall() = Ball(Point(1, -1), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    world->mutableBall() = Ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    EXPECT_TRUE(result);
}

TEST(
    FunctionValidatorTest,
    test_validation_function_with_two_loops_fails_if_loop_conditions_are_not_satisfied_in_order)
{
    // This validation function will only pass if the ball's x-coordinate becomes positive
    // before the ball's y-coordinate becomes positive
    ValidationFunction validation_function = [](std::shared_ptr<World> world,
                                                ValidationCoroutine::push_type& yield) {
        while (world->ball().position().x() < 0)
        {
            yield();
        }

        while (world->ball().position().y() < 0)
        {
            yield();
        }
    };

    auto world = std::make_shared<World>(::Test::TestUtil::createBlankTestingWorld());
    FunctionValidator function_validator(validation_function, world);

    world->mutableBall() = Ball(Point(-1, -1), Vector(0, 0), Timestamp::fromSeconds(0));
    bool result          = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    // Make the ball's y-coordinate positive before the x-coordinate
    world->mutableBall() = Ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    EXPECT_FALSE(result);

    world->mutableBall() = Ball(Point(1, 1), Vector(0, 0), Timestamp::fromSeconds(0));
    result               = function_validator.executeAndCheckForSuccess();
    // Even though both conditions (x > 0 and y > 0) are met in this state, they were not
    // met in the order the validation function wanted so the function has not succeeded /
    // passed
    EXPECT_FALSE(result);
}
