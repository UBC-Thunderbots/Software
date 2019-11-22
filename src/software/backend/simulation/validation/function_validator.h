#pragma once

#include "software/backend/simulation/validation/validation_function.h"

/**
 * This class is a wrapper to make it easier to work with ValidationFunctions. It provides
 * an easy way to manage the coroutines required to run ValidationFunctions as well as see
 * if the function has succeeded / passed.
 */
class FunctionValidator
{
   public:
    /**
     * Creates a new FunctionValidator.
     *
     * @param validation_function The ValidationFunction this FunctionValidator should
     * manage and run
     * @param world The world that will be given to the ValidationFunction in order to run
     * it
     */
    explicit FunctionValidator(const ValidationFunction& validation_function,
                               std::shared_ptr<World> world);

    /**
     * Runs the ValidationFunction that was given to this FunctionValidator on
     * construction and returns true if the ValidationFunction has succeeded / passed.
     * Returns false otherwise. Once the ValidationFunction has succeeded (returned true
     * once), the coroutine will not be restarted and this function will always return
     * true without re-running the ValidationFunction.
     *
     * @return true if the internal ValidationFunction has succeeded / passed, and false
     * otherwise
     */
    bool executeAndCheckForSuccess();

   private:
    /**
     * A wrapper function for the validation_function.
     *
     * This function exists because when the coroutine (validation_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * validation_sequence function to be run once which may cause unexpected results.
     * Additionally, it is easier to bind the coroutine to a wrapper function and have the
     * wrapper function pass any arguments to the validation_function rather than pass
     * all parameters through coroutines.
     *
     * @param yield The coroutine push_type for the validation_function
     */
    void executeAndCheckForSuccessWrapper(ValidationCoroutine::push_type& yield);

    // The coroutine that will be given to the validation function
    ValidationCoroutine::pull_type validation_sequence;

    // The world that will be given to the validation_function when it is executed
    std::shared_ptr<World> world;

    // The validation function being executed / managed
    ValidationFunction validation_function;
};
