#pragma once

#include <boost/coroutine2/all.hpp>
#include <functional>

#include "software/world/world.h"

struct ConditionAndErrorMessage
{
    // Condition that if true means that the world is in a passing state
    std::function<bool(std::shared_ptr<World>)> condition;
    // Error message for if the condition is false and the test is failing
    std::string error_message;
};

using ValidationCoroutine = boost::coroutines2::coroutine<void>;
using ValidationFunction =
    std::function<void(std::shared_ptr<World>, ValidationCoroutine::push_type&)>;

using TerminatingValidationCoroutine = boost::coroutines2::coroutine<std::string>;
using TerminatingValidationFunction  = std::function<void(
    std::shared_ptr<World>, TerminatingValidationCoroutine::push_type&)>;

/**
 * Creates a terminating validation function that check conditions in a sequence, i.e.
 * condition at index 1 must pass before condition at index 2 is checked. If the condition
 * is not true, yields the associated error message
 *
 * @param conditions_and_error_messages Pairs of conditions and error messages
 *
 * @return The terminating validation function
 */
TerminatingValidationFunction createTerminatingValidationFunction(
    std::vector<ConditionAndErrorMessage> conditions_and_error_messages);

/**
 * Creates a non-terminating validation function that checks if a condition is true and if
 * not adds a gtest FAIL() with the given error message
 *
 * @param conditions_and_error_message Pair of conditions and error messages should the
 * condition not pass
 *
 * @return The non-terminating validation function
 */
ValidationFunction createNonTerminatingValidationFunction(
    ConditionAndErrorMessage conditions_and_error_message);
