#include "software/ai/hl/stp/tactic/tactic.h"

#include "software/logger/logger.h"

Tactic::Tactic(bool loop_forever,
               const std::set<RobotCapabilities::Capability> &capability_reqs_)
    : action_sequence(boost::bind(&Tactic::calculateNextActionWrapper, this, _1)),
      done_(false),
      loop_forever(loop_forever),
      capability_reqs(capability_reqs_)
{
    // require movement capability by default
    capability_reqs.emplace(RobotCapabilities::Capability::Move);
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

bool Tactic::isGoalieTactic() const
{
    return false;
}

std::shared_ptr<Action> Tactic::getNextAction(void)
{
    std::shared_ptr<Action> next_action = nullptr;
    if (!robot)
    {
        LOG(WARNING) << "Requesting the next Action for a Tactic without a Robot assigned"
                     << std::endl;
    }
    else
    {
        // We call the getNextActionHelper before checking if we should loop forever
        // so we can catch the tactic right when it's done. Since we do not want to return
        // any nullptrs while a tactic is looping forever, we need to perform this
        // check after running the logic and immediately restarting.
        next_action = getNextActionHelper();
        if (done_ && loop_forever)
        {
            // Re-start the action sequence by re-creating it
            action_sequence = ActionCoroutine::pull_type(
                boost::bind(&Tactic::calculateNextActionWrapper, this, _1));
            next_action = getNextActionHelper();
        }
    }

    return next_action;
}

void Tactic::calculateNextActionWrapper(ActionCoroutine::push_type &yield)
{
    // Yield a null pointer the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield(std::shared_ptr<Action>{});

    // Anytime after the first function call, the calculateNextAction function will be
    // used to perform the real logic. The calculateNextAction function will yield its
    // values to the top of the coroutine stack, where they will be retrieved by
    // getNextAction, so we do not need to yield or return the result of this function
    calculateNextAction(yield);
}

std::shared_ptr<Action> Tactic::getNextActionHelper()
{
    std::shared_ptr<Action> next_action = nullptr;
    // Check the coroutine status to see if it has any more work to do.
    if (action_sequence)
    {
        // Run the coroutine. This will call the bound calculateNextAction function
        action_sequence();

        // Check if the coroutine is still valid before getting the result. This makes
        // sure we don't try get the result after "running out the bottom" of the
        // coroutine function
        if (action_sequence)
        {
            // Extract the result from the coroutine. This will be whatever value was
            // yielded by the calculateNextAction function
            next_action = action_sequence.get();
        }
    }

    // The Tactic is considered done once the next_action becomes a nullptr. This could
    // either be because it was returned by the calculateNextAction function, or because
    // the action_sequence coroutine is done and has no more work to do.
    done_ = !static_cast<bool>(next_action);

    return next_action;
}

const std::set<RobotCapabilities::Capability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapabilities::Capability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}
