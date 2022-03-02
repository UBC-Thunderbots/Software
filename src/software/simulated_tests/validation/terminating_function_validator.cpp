#include "software/simulated_tests/validation/terminating_function_validator.h"

#include <boost/bind.hpp>

TerminatingFunctionValidator::TerminatingFunctionValidator(
    ValidationFunction validation_function,
    std::shared_ptr<World> world)
    :  // We need to provide the world and validation_function in the coroutine function
       // binding so that the wrapper function has access to the correct variable context,
       // otherwise the World inside the coroutine will not update properly when the
       // pointer is updated, and the wrong validation_function may be run.
      validation_sequence(
          boost::bind(&TerminatingFunctionValidator::executeAndCheckForSuccessWrapper,
                      this, _1, world, validation_function)),
      current_error_message("")
{
}

std::string TerminatingFunctionValidator::currentErrorMessage() const
{
    return current_error_message;
}

void TerminatingFunctionValidator::executeAndCheckForSuccessWrapper(
    ValidationCoroutine::push_type &yield, std::shared_ptr<World> world,
    ValidationFunction validation_function)
{
    // Yield the very first time the function is called, so that the validation_function
    // is not run until this coroutine / wrapper function is called again by
    // executeAndCheckForSuccess
    yield("");

    // Anytime after the first function call, the validation_function will be
    // used to perform the real logic.
    validation_function(world, yield);
}

bool TerminatingFunctionValidator::executeAndCheckForSuccess()
{
    // Check the coroutine status to see if it has any more work to do.
    if (validation_sequence)
    {
        // Run the coroutine. This will call the bound executeAndCheckForSuccessWrapper
        // function
        validation_sequence();
        current_error_message = validation_sequence.get();
    }
    else
    {
        current_error_message = "";
    }

    // The validation_function is done if the coroutine evaluates to false, which means
    // execution has "dropped out" the bottom of the function and there is no more work to
    // do. If this is the case then the validation_function has passed successfully
    return !static_cast<bool>(validation_sequence);
}
