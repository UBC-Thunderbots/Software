#include "ai/hl/stp/tactic/tactic.h"

#include "util/logger/init.h"

Tactic::Tactic(bool loop_forever, RobotCapabilityFlags capability_reqs_)
    : intent_sequence(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1)),
      done_(false),
      loop_forever(loop_forever),
      capability_reqs(capability_reqs_)
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
        // We call the getNextIntentHelper before checking if we should loop forever
        // so we can catch the tactic right when it's done. Since we do not want to return
        // any nullptrs while a tactic is looping forever, we need to perform this
        // check after running the logic and immediately restarting.
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
    // Check the coroutine status to see if it has any more work to do.
    if (intent_sequence)
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

    // The Tactic is considered done once the next_intent becomes a nullptr. This could
    // either be because it was returned by the calculateNextIntent function, or because
    // the intent_sequence coroutine is done and has no more work to do.
    done_ = !static_cast<bool>(next_intent);

    return next_intent;
}

const RobotCapabilityFlags &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

RobotCapabilityFlags &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}
