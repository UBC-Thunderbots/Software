#include "ai/hl/stp/tactic/tactic.h"

#include "util/logger/init.h"

Tactic::Tactic()
    : intent_sequence(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1)),
      done_(false)
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

    if (intent_sequence)
    {
        // Calculate and return the next Intent
        intent_sequence();
        auto next_intent = intent_sequence.get();
        // The tactic is considered done once a nullptr is returned
        // We set the done_ variable here in addition to below (once the coroutine is
        // done) to make sure that the done() function will always return true at the same
        // time a nullptr is returned (which also indicates the Tactic is done)
        done_ = !static_cast<bool>(next_intent);
        return next_intent;
    }
    // If the coroutine "iterator" is done, the calculateNextIntent function has completed
    // and has no more work to do. Therefore, the Tactic is done so we set done_ to true
    // and return a nullptr
    done_ = true;
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
