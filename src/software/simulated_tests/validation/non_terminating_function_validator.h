#pragma once

#include "software/simulated_tests/validation/validation_function.h"

/**
 * This class is a wrapper to make it easier to work with
 * NonTerminatingValidationFunctions. It provides an easy way to manage the coroutines
 * required to run NonTerminatingValidationFunctions as well as see if the function has
 * succeeded / passed.
 *
 * This class will run the provided coroutine continuously by restarting it every time the
 * coroutine has completed.
 */
class NonTerminatingFunctionValidator
{
   public:
    /**
     * Creates a new NonTerminatingFunctionValidator.
     *
     * @param validation_function The NonTerminatingValidationFunction this
     * NonTerminatingFunctionValidator should manage and run
     * @param world The world that will be given to the NonTerminatingValidationFunction
     * in order to run it
     */
    explicit NonTerminatingFunctionValidator(
        NonTerminatingValidationFunction validation_function,
        std::shared_ptr<World> world);

    /**
     * Runs the NonTerminatingValidationFunction that was given to this
     * NonTerminatingFunctionValidator on construction. The
     * NonTerminatingValidationFunction will be restarted if it has completed. As such,
     * the NonTerminatingValidationFunction can never be "done" and will only terminate
     * due to failures within the NonTerminatingValidationFunction, such as a failed
     * GoogleTest assert
     */
    void executeAndCheckForFailures();

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
     * @param world The world that will be given to the validation_function being run.
     * Because it's a shared_ptr any external changes made to the world will be reflected
     * inside the validation_function.
     * @param validation_function The validation_function to run in the coroutine
     */
    void executeAndCheckForFailuresWrapper(
        NonTerminatingValidationCoroutine::push_type& yield, std::shared_ptr<World> world,
        NonTerminatingValidationFunction validation_function);

    // The coroutine that will be given to the validation function
    NonTerminatingValidationCoroutine::pull_type validation_sequence;
    std::shared_ptr<World> world_;
    NonTerminatingValidationFunction validation_function_;
};
