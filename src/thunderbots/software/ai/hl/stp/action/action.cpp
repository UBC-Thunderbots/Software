#include "ai/hl/stp/action/action.h"

#include "util/logger/init.h"

Action::Action()
    : intent_sequence(boost::bind(&Action::calculateNextIntentWrapper, this, _1))
{
}

bool Action::done() const
{
    return !static_cast<bool>(intent_sequence);
}

std::unique_ptr<Intent> Action::getNextIntent()
{
    std::unique_ptr<Intent> next_intent = {};
    if (!robot)
    {
        LOG(WARNING)
            << "Requesting the next Intent for an Action without a Robot assigned"
            << std::endl;
    }
    // Default to returning a null pointer if the coroutine "iterator" is done, as this
    // means that the calculateNextIntent function has completed and therefore the
    // Action is done
    else if (intent_sequence)
    {
        // Get the next intent from the coroutine
        next_intent = intent_sequence.get();

        // Continue to run the coroutine (basically setting up for the next time this
        // function is called)
        intent_sequence();
    }

    return next_intent;
}

std::unique_ptr<Intent> Action::calculateNextIntentWrapper(
    IntentCoroutine::push_type &yield)
{
    // Yield a null pointer the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield(std::unique_ptr<Intent>{});

    // Anytime after the first function call, the calculateNextIntent function will be
    // used to perform the real logic. Note that we need to "yield" up at each level of
    // of the coroutine
    yield(calculateNextIntent(yield));
}