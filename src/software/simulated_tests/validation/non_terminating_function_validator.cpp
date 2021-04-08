#include "software/simulated_tests/validation/non_terminating_function_validator.h"

#include <gtest/gtest.h>

#include <boost/bind.hpp>

NonTerminatingFunctionValidator::NonTerminatingFunctionValidator(
    ValidationFunction validation_function, std::shared_ptr<World> world)
    :  // We need to provide the world and validation_function in the coroutine function
       // binding so that the wrapper function has access to the correct variable context,
       // otherwise the World inside the coroutine will not update properly when the
       // pointer is updated, and the wrong validation_function may be run.
      validation_sequence(
          boost::bind(&NonTerminatingFunctionValidator::executeAndCheckForFailuresWrapper,
                      this, _1, world, validation_function)),
      world_(world),
      validation_function_(validation_function)
{
}

std::optional<std::string> NonTerminatingFunctionValidator::executeAndCheckForFailures()
{
    // Check the coroutine status to see if it has any more work to do.
    if (!validation_sequence)
    {
        // Re-start the coroutine by re-creating it
        validation_sequence = ValidationCoroutine::pull_type(boost::bind(
            &NonTerminatingFunctionValidator::executeAndCheckForFailuresWrapper, this, _1,
            world_, validation_function_));
    }

    // Run the coroutine. This will call the bound executeAndCheckForFailuresWrapper
    // function
    validation_sequence();
    std::string error_msg = validation_sequence.get();
    if (error_msg != "")
    {
        return error_msg;
    }
    else
    {
        return std::nullopt;
    }
}

void NonTerminatingFunctionValidator::executeAndCheckForFailuresWrapper(
    ValidationCoroutine::push_type &yield, std::shared_ptr<World> world,
    ValidationFunction validation_function)
{
    // Yield the very first time the function is called, so that the validation_function
    // is not run until this coroutine / wrapper function is called again by
    // executeAndCheckForFailures
    yield("");

    // Anytime after the first function call, the validation_function will be
    // used to perform the real logic.
    validation_function(world, yield);
}
