#include "ai/hl/stp/action/action.h"

Action::Action(const Robot &robot)
    : robot(robot),
      intent_sequence(boost::bind(&Action::calculateNextIntentWrapper, this, _1))
{
}

bool Action::done() const
{
    return !static_cast<bool>(intent_sequence);
}

std::unique_ptr<Intent> Action::getNextIntent()
{
    // If the coroutine "iterator" is done, the calculateNextIntent function has completed
    // and therefore the Action is done, so we return a null pointer
    if (intent_sequence)
    {
        // Calculate and return the next Intent
        intent_sequence();
        auto next_intent = intent_sequence.get();
        return next_intent;
    }
    return std::unique_ptr<Intent>{};
}

std::unique_ptr<Intent> Action::calculateNextIntentWrapper(
    intent_coroutine::push_type &yield)
{
    // Yield a null pointer the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield(std::unique_ptr<Intent>{});

    // Anytime after the first function call, the calculateNextIntent function will be
    // used to perform the real logic
    return calculateNextIntent(yield);
}