#include "software/simulated_tests/validation/continuous_function_validator.h"

#include <boost/bind.hpp>

ContinuousFunctionValidator::ContinuousFunctionValidator(
    ValidationFunction validation_function, std::shared_ptr<World> world)
    : world_(world),
      validation_function_(validation_function),
      // We need to provide the world and validation_function in the coroutine function
      // binding so that the wrapper function has access to the correct variable context,
      // otherwise the World inside the coroutine will not update properly when the
      // pointer is updated, and the wrong validation_function may be run.
      validation_sequence(
          boost::bind(&ContinuousFunctionValidator::executeAndCheckForFailuresWrapper,
                      this, _1, world, validation_function))
{
}

void ContinuousFunctionValidator::executeAndCheckForFailures()
{
    // Check the coroutine status to see if it has any more work to do.
    if (!validation_sequence)
    {
        // Re-start the coroutine by re-creating it
        validation_sequence = ValidationCoroutine::pull_type(
            boost::bind(&ContinuousFunctionValidator::executeAndCheckForFailuresWrapper,
                        this, _1, world_, validation_function_));
    }

    // Run the coroutine. This will call the bound executeAndCheckForFailuresWrapper
    // function
    validation_sequence();
}

void ContinuousFunctionValidator::executeAndCheckForFailuresWrapper(
    ValidationCoroutine::push_type &yield, std::shared_ptr<World> world,
    ValidationFunction validation_function)
{
    // Yield the very first time the function is called, so that the validation_function
    // is not run until this coroutine / wrapper function is called again by
    // executeAndCheckForFailures
    yield();

    // Anytime after the first function call, the validation_function will be
    // used to perform the real logic.
    validation_function(world, yield);
}
