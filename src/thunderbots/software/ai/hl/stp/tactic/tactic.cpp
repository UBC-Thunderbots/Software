#include "ai/hl/stp/tactic/tactic.h"

#include "util/logger/init.h"

Tactic::Tactic(bool loop_forever)
    : intent_sequence(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1)),
      done_(false),
      loop_forever(loop_forever)
{
}

bool Tactic::done() const
{
    return done_;
}

std::optional<Robot> Tactic::getAssignedRobot() const
{
    return robot;
}

void Tactic::updateRobot(const Robot &robot)
{
    this->robot = robot;
}

std::unique_ptr<Intent> Tactic::getNextIntent()
{
    std::unique_ptr<Intent> next_intent = nullptr;
    if (!robot)
    {
        LOG(WARNING) << "Requesting the next Intent for a Tactic without a Robot assigned"
                     << std::endl;
    }
    else
    {
        next_intent = getNextIntentHelper();
        if (done_ && loop_forever)
        {
            // Re-start the intent sequence by re-creating it
            intent_sequence = IntentCoroutine::pull_type(
                boost::bind(&Tactic::calculateNextIntentWrapper, this, _1));
            next_intent = getNextIntentHelper();
        }
    }

    return next_intent;
}

void Tactic::calculateNextIntentWrapper(IntentCoroutine::push_type &yield)
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

std::unique_ptr<Intent> Tactic::getNextIntentHelper()
{
    std::unique_ptr<Intent> next_intent = nullptr;
    // Check if the coroutine "iterator" has any more work to do. Only run the coroutine
    // if there is work to be done otherwise the coroutine library will fail on an assert.
    if (intent_sequence())
    {
        // Extract the result from the coroutine. This will be whatever value was
        // yielded by the calculateNextIntent function
        next_intent = intent_sequence.get();
    }

    // The Tactic is considered done once the next_intent becomes a nullptr. This could
    // either be because it was returned by the calculateNextIntent function, or because
    // the intent_sequence coroutine is done and has no more work to do.
    done_ = !static_cast<bool>(next_intent);

    return next_intent;
}
