#include "ai/hl/stp/tactic/tactic.h"

#include "util/logger/init.h"

Tactic::Tactic(bool loop_forever)
    : intent_sequence(std::make_unique<intent_coroutine::pull_type>(
          [this](intent_coroutine::push_type &yield) {
              return this->calculateNextIntentWrapper(yield);
          })),
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
        // If the tactic is done and is supposed to loop forever, we re-create the
        // intent_sequence which "restarts" the coroutine. We then run the coroutine
        // again, and return the result from the restarted coroutine rather than the
        // old one. This way, any callers of this function won't accidentally get a
        // nullptr returned for a single call (which could come from the "old" coroutine)
        // when this Tactic restarts
        intent_sequence.reset();
        intent_sequence = std::make_unique<intent_coroutine::pull_type>(
            [this](intent_coroutine::push_type &yield) {
                return this->calculateNextIntentWrapper(yield);
            });
        next_intent = getNextIntentHelper();
    }

    return next_intent;
}

std::unique_ptr<Intent> Tactic::calculateNextIntentWrapper(
    intent_coroutine::push_type &yield)
{
    // Yield a null pointer the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield(std::unique_ptr<Intent>{});

    // Anytime after the first function call, the calculateNextIntent function will be
    // used to perform the real logic
    return calculateNextIntent(yield);
}

std::unique_ptr<Intent> Tactic::getNextIntentHelper()
{
    std::unique_ptr<Intent> next_intent;
    // Check if the coroutine "iterator" has any more work to do. Only run the coroutine
    // if there is work to be done otherwise the coroutine library will fail on an assert.
    if (*intent_sequence)
    {
        // Run the coroutine
        (*intent_sequence)();
        // Get the result of running the coroutine, which is the next Intent the Tactic
        // wants to run
        next_intent = intent_sequence->get();
    }

    // The Tactic is considered done once the next_intent becomes a nullptr. This could
    // either be because it was returned by the calculateNextIntent function, or because
    // the intent_sequence coroutine is done and has no more work to do.
    done_ = !static_cast<bool>(next_intent);

    return next_intent;
}
