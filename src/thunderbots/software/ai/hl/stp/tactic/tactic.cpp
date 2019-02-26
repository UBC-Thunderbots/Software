#include "ai/hl/stp/tactic/tactic.h"

#include "util/logger/init.h"

Tactic::Tactic()
    : intent_sequence(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1)),
      done_(false)
{
}

bool Tactic::done() const
{
    // If the coroutine "iterator" is done (ie. evaluates to false, has no more values
    // to iterate), the calculateNextIntent function has completed and therefore
    // the Tactic is done
    //
    // We also check out internal done_ variable. This is set automatically by the
    // getNextIntent function if we ever return a nullptr. We do this extra check because
    // Tactics can return a nullptr one "tick" (aka function call) before the coroutine
    // actually evaluates as done. This way we ensure that the Tactic::done() function
    // will always evaluate to true on the exact same call a nullptr is returned.
    //
    // This is important since higher-level functions rely on the done() function, and
    // returning nullptr values "out of sync" with this could cause problems
    return done_ || !static_cast<bool>(intent_sequence);
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

    // If the coroutine "iterator" is done, the calculateNextIntent function has completed
    // and therefore the Tactic is done, so we return a null pointer
    if (intent_sequence)
    {
        // Calculate and return the next Intent
        intent_sequence();
        auto next_intent = intent_sequence.get();
        // The tactic is considered done once a nullptr is returned
        done_ = !static_cast<bool>(next_intent);
        return next_intent;
    }
    return std::unique_ptr<Intent>{};
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
