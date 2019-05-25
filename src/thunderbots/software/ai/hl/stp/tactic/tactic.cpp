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
    if (!robot)
    {
        LOG(WARNING) << "Requesting the next Intent for a Tactic without a Robot assigned"
                     << std::endl;
        return std::unique_ptr<Intent>{};
    }

    auto next_intent = getNextIntentHelper();
    if (done_ && loop_forever)
    {
        // Re-start the intent sequence
        intent_sequence = IntentCoroutine::pull_type(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1));
        next_intent = getNextIntentHelper();
    }

    return next_intent;
}

std::unique_ptr<Intent> Tactic::calculateNextIntentWrapper(
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

std::unique_ptr<Intent> Tactic::getNextIntentHelper()
{
    std::unique_ptr<Intent> next_intent = {};
    // Check if the coroutine "iterator" has any more work to do. Only run the coroutine
    // if there is work to be done otherwise the coroutine library will fail on an assert.
    if (intent_sequence)
    {
        // Get the result of running the coroutine, which is the next Intent the Tactic
        // wants to run
        next_intent = intent_sequence.get();

        // Continue to run the coroutine (basically setting up for the next time this
        // function is called)
        intent_sequence();
    }

    // The Tactic is considered done once the next_intent becomes a nullptr. This could
    // either be because it was returned by the calculateNextIntent function, or because
    // the intent_sequence coroutine is done and has no more work to do.
    done_ = !static_cast<bool>(next_intent);

    return next_intent;
}
