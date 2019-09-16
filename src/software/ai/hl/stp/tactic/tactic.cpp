#include "software/ai/hl/stp/tactic/tactic.h"

#include <g3log/g3log.hpp>

Tactic::Tactic(bool loop_forever,
               const std::set<RobotCapabilities::Capability> &capability_reqs_)
    : intent_sequence(boost::bind(&Tactic::calculateNextIntentWrapper, this, _1)),
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

std::unique_ptr<Intent> Tactic::getNextIntent(
    const std::optional<GameState> &game_state_opt)
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

    if (next_intent != nullptr)
    {
        // Figure out where this intent should and should not move to, based on the
        // world state, whitelisted avoid areas, and blacklisted avoid areas
        std::vector<AvoidArea> avoid_areas;
        if (game_state_opt)
        {
            avoid_areas = getAreasToAvoid(*game_state_opt);
        }
        avoid_areas.insert(avoid_areas.end(), blacklisted_avoid_areas.begin(),
                           blacklisted_avoid_areas.end());

        next_intent->setAreasToAvoid(avoid_areas);
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

const std::set<RobotCapabilities::Capability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapabilities::Capability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}


void Tactic::addWhitelistedAvoidArea(AvoidArea area)
{
    whitelisted_avoid_areas.emplace_back(area);
}

void Tactic::addBlacklistedAvoidArea(AvoidArea area)
{
    // Only add this area to our list if it's not already present
    if (std::find(blacklisted_avoid_areas.begin(), blacklisted_avoid_areas.end(), area) ==
        blacklisted_avoid_areas.end())
    {
        blacklisted_avoid_areas.emplace_back(area);
    }
}

void Tactic::removeBlacklistedAvoidArea(AvoidArea area)
{
    blacklisted_avoid_areas.erase(
        std::remove(blacklisted_avoid_areas.begin(), blacklisted_avoid_areas.end(), area),
        blacklisted_avoid_areas.end());
}

std::vector<AvoidArea> Tactic::getAreasToAvoid(const GameState &game_state)
{
    std::vector<AvoidArea> areas_to_avoid;

    // Checks if the given area is in the whitelist and adds it to
    // the list to return if not
    auto addAreaIfNotInWhitelist = [&](AvoidArea area) {
        bool area_is_whitelisted =
            std::find(whitelisted_avoid_areas.begin(), whitelisted_avoid_areas.end(),
                      area) != whitelisted_avoid_areas.end();
        bool area_is_already_added =
            std::find(areas_to_avoid.begin(), areas_to_avoid.end(), area) !=
            areas_to_avoid.end();
        if (!area_is_whitelisted && !area_is_already_added)
        {
            areas_to_avoid.emplace_back(area);
        }
    };

    if (game_state.stayAwayFromBall())
    {
        addAreaIfNotInWhitelist(AvoidArea::HALF_METER_AROUND_BALL);
    }

    if (game_state.isPenalty())
    {
        if (game_state.isOurPenalty())
        {
            addAreaIfNotInWhitelist(AvoidArea::ENEMY_HALF);
        }
        else
        {
            // Is their penalty
            addAreaIfNotInWhitelist(AvoidArea::FRIENDLY_HALF);
        }
    }
    else if (game_state.isKickoff())
    {
        addAreaIfNotInWhitelist(AvoidArea::HALF_METER_AROUND_BALL);
        addAreaIfNotInWhitelist(AvoidArea::CENTER_CIRCLE);
        addAreaIfNotInWhitelist(AvoidArea::ENEMY_HALF);
    }
    else
    {
        if (game_state.stayAwayFromBall() || game_state.isOurKickoff())
        {
            addAreaIfNotInWhitelist(AvoidArea::HALF_METER_AROUND_BALL);
        }

        if (game_state.isOurPenalty())
        {
            addAreaIfNotInWhitelist(AvoidArea::ENEMY_DEFENSE_AREA);
        }
        else
        {
            addAreaIfNotInWhitelist(AvoidArea::INFLATED_ENEMY_DEFENSE_AREA);
        }
    }

    addAreaIfNotInWhitelist(AvoidArea::FRIENDLY_DEFENSE_AREA);
    addAreaIfNotInWhitelist(AvoidArea::BALL);

    return areas_to_avoid;
}
