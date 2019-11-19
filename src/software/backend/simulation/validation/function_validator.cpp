#include "software/backend/simulation/validation/function_validator.h"
#include <boost/bind.hpp>

FunctionValidator::FunctionValidator(const ValidationFunction &validation_function, std::shared_ptr<World> world) :
validation_function(validation_function),
world(world),
validation_sequence(boost::bind(&FunctionValidator::executeAndCheckForSuccessWrapper, this, _1))
{
}

void FunctionValidator::executeAndCheckForSuccessWrapper(ValidationCoroutine::push_type &yield) {
    // Yield the very first time the function is called, so that the validation_function is not run
    // until this coroutine / wrapper function is called again by executeAndCheckForSuccess
    yield();

    // Anytime after the first function call, the validation_function will be
    // used to perform the real logic.
    validation_function(world, yield);
}

bool FunctionValidator::executeAndCheckForSuccess() {
    // Check the coroutine status to see if it has any more work to do.
    if(validation_sequence) {
        // Run the coroutine. This will call the bound executeAndCheckForSuccessWrapper function
        validation_sequence();
    }

    // The validation_function is done if the coroutine evaluates to false, which means execution
    // has "dropped out" the bottom of the function and there is no more work to do. If this is the
    // case then the validation_function has passed successfully
    return !static_cast<bool>(validation_sequence);
}
