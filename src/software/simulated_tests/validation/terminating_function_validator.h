#pragma once

#include "software/simulated_tests/validation/validation_function.h"

/**
 * This class is a wrapper to make it easier to work with TerminatingValidationFunctions.
 * It provides an easy way to manage the coroutines required to run
 * TerminatingValidationFunctions as well as see if the function has succeeded / passed.
 *
 * This class will run the provided coroutine a single time and stop once it has
 * completed. The coroutine will NOT be restarted upon completion.
 */
class TerminatingFunctionValidator
{
   public:
    /**
     * Creates a new TerminatingFunctionValidator.
     *
     * @param validation_function The TerminatingValidationFunction this
     * TerminatingFunctionValidator should manage and run
     * @param world The world that will be given to the TerminatingValidationFunction in
     * order to run it
     */
    explicit TerminatingFunctionValidator(
        TerminatingValidationFunction validation_function, std::shared_ptr<World> world);

    /**
     * Runs the TerminatingValidationFunction that was given to this
     * TerminatingFunctionValidator on construction and returns true if the
     * TerminatingValidationFunction has succeeded / passed. Returns false otherwise. Once
     * the TerminatingValidationFunction has succeeded (returned true once), the coroutine
     * will not be restarted and this function will always return true without re-running
     * the TerminatingValidationFunction.
     *
     * @return true if the internal TerminatingValidationFunction has succeeded / passed,
     * and false otherwise
     */
    bool executeAndCheckForSuccess();

    std::string currentErrorMessage();

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
    void executeAndCheckForSuccessWrapper(
        TerminatingValidationCoroutine::push_type& yield, std::shared_ptr<World> world,
        TerminatingValidationFunction validation_function);

    // The coroutine that will be given to the validation function
    TerminatingValidationCoroutine::pull_type validation_sequence;

    std::string current_error_message;
};
