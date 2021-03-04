#include "software/simulated_tests/validation/validation_function.h"

#include <gtest/gtest.h>

TerminatingValidationFunction createTerminatingValidationFunction(
    std::vector<ConditionAndErrorMessage> conditions_and_error_messages)
{
    TerminatingValidationFunction validation_function =
        [&conditions_and_error_messages](
            std::shared_ptr<World> world,
            TerminatingValidationCoroutine::push_type& yield) {
            for (auto& c : conditions_and_error_messages)
            {
                while (!c.condition(world))
                {
                    yield(c.error_message);
                }
            }
        };
    return validation_function;
}

ValidationFunction createNonTerminatingValidationFunction(
    ConditionAndErrorMessage conditions_and_error_message)
{
    ValidationFunction validation_function = [&conditions_and_error_message](
                                                 std::shared_ptr<World> world,
                                                 ValidationCoroutine::push_type& yield) {
        if (!conditions_and_error_message.condition(world))
        {
            FAIL() << conditions_and_error_message.error_message;
        }
    };
    return validation_function;
}
