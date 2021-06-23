#include "software/ai/hl/stp/tactic/tactic.h"

#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

Tactic::Tactic(bool loop_forever, const std::set<RobotCapability> &capability_reqs_)
    : action_sequence(boost::bind(&Tactic::calculateNextActionWrapper, this, _1)),
      done_(false),
      intent(),
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
    return robot_;
}

void Tactic::updateRobot(const Robot &robot)
{
    this->robot_ = robot;
}

bool Tactic::isGoalieTactic() const
{
    return false;
}

std::shared_ptr<Action> Tactic::getNextAction(void)
{
    std::shared_ptr<Action> next_action = nullptr;
    if (!robot_)
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

const std::set<RobotCapability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

std::unique_ptr<Intent> Tactic::get(const Robot &robot, const World &world)
{
    // TODO (#1888): remove updateWorldParams and updateRobot
    updateWorldParams(world);
    updateRobot(robot);

    updateIntent(TacticUpdate(robot, world, [this](std::unique_ptr<Intent> new_intent) {
        intent = std::move(new_intent);
    }));

    if (intent)
    {
        return std::move(intent);
    }
    else
    {
        return std::make_unique<StopIntent>(robot.id(), false);
    }
}

void Tactic::updateIntent(const TacticUpdate &tactic_update)
{
    // Try to get an intent from the tactic
    std::shared_ptr<Action> action = getNextAction();
    std::unique_ptr<Intent> intent;
    if (action)
    {
        action->updateWorldParams(tactic_update.world);
        tactic_update.set_intent(action->getNextIntent());
    }
}
std::string Tactic::getAdditionalInfo() const
{
    // do nothing by default
    return "";
}

std::string Tactic::parseCondensedFsmState(std::string fsm_state)const
{
    auto start_angle_bracket_pos = fsm_state.find_last_of('<');
    if (start_angle_bracket_pos == std::string::npos)
    {
        start_angle_bracket_pos = 0;
    }
    auto end_angle_bracket_pos = fsm_state.find_first_of('>');
    if (end_angle_bracket_pos == std::string::npos)
    {
        end_angle_bracket_pos = fsm_state.size() - 1;
    }
    auto inside_innermost_angle_brackets_str = fsm_state.substr(
        start_angle_bracket_pos + 1, end_angle_bracket_pos - start_angle_bracket_pos - 1);
    auto last_colon_pos = inside_innermost_angle_brackets_str.find_last_of(':');
    if (last_colon_pos == std::string::npos)
    {
        return inside_innermost_angle_brackets_str;
    }
    else
    {
        auto ret_str = inside_innermost_angle_brackets_str.substr(last_colon_pos + 1);
        return ret_str;
    }
}
