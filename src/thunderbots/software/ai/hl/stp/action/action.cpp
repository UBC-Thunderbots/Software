#include "software/ai/hl/stp/action/action.h"

#include <g3log/g3log.hpp>

Action::Action()
    : intent_sequence(boost::bind(&Action::calculateNextIntentWrapper, this, _1))
{
}

bool Action::done() const
{
    // The action is done if the coroutine evaluates to false, which means execution
    // has "dropped out" the bottom of the function and there is no more work to do
    return !static_cast<bool>(intent_sequence);
}

std::unique_ptr<Intent> Action::getNextIntent()
{
    std::unique_ptr<Intent> next_intent = nullptr;
    if (!robot)
    {
        LOG(WARNING)
            << "Requesting the next Intent for an Action without a Robot assigned"
            << std::endl;
    }
    // Check the coroutine status to see if it has any more work to do.
    else if (intent_sequence)
    {
        // Run the coroutine. This will call the bound calculateNextIntent function
        intent_sequence();

        // Check if the coroutine is still valid before getting the result. This makes
        // sure we don't try get the result after "running out the bottom" of the
        // coroutine function
        if (intent_sequence)
        {
            // Extract the result from the coroutine. This will be whatever value was
            // yielded by the calculateNextIntent function
            next_intent = intent_sequence.get();
        }
    }

    return next_intent;
}

void Action::calculateNextIntentWrapper(IntentCoroutine::push_type &yield)
{
    // Yield a null pointer the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield(std::unique_ptr<Intent>{});

    // Anytime after the first function call, the calculateNextIntent function will be
    // used to perform the real logic. The calculateNextIntent function will yield its
    // values to the top of the coroutine stack, where they will be retrieved by
    // getNextIntent, so we do not need to yield or return the result of this function
    calculateNextIntent(yield);
}
