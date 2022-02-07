#include "software/ai/hl/stp/play/play.h"

Play::Play(std::shared_ptr<const AiConfig> ai_config, bool requires_goalie)
    : ai_config(ai_config),
      requires_goalie(requires_goalie),
      tactic_sequence(boost::bind(&Play::getNextTacticsWrapper, this, _1)),
      world(std::nullopt)
{
}

PriorityTacticVector Play::getTactics(const World &world)
{
    // Update the member variable that stores the world. This will be used by the
    // getNextTacticsWrapper function (inside the coroutine) to pass the World data to
    // the getNextTactics function. This is easier than directly passing the World data
    // into the coroutine
    this->world = world;
    // Check the coroutine status to see if it has any more work to do.
    if (tactic_sequence)
    {
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }

    // Check if the coroutine is still valid before getting the result. This makes
    // sure we don't try get the result after "running out the bottom" of the
    // coroutine function
    if (tactic_sequence)
    {
        // Extract the result from the coroutine. This will be whatever value was
        // yielded by the getNextTactics function
        auto next_tactics = tactic_sequence.get();
        return next_tactics;
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
        if (tactic_sequence)
        {
            // Extract the result from the coroutine. This will be whatever value was
            // yielded by the getNextTactics function
            auto next_tactics = tactic_sequence.get();
            return next_tactics;
        }
        else
        {
            LOG(WARNING) << "Failed to restart play" << std::endl;
        }
    }
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done so we return an empty
    // vector
    return PriorityTacticVector();
}

std::vector<std::unique_ptr<Intent>> Play::get(
    RobotToTacticAssignmentFunction robot_to_tactic_assignment_algorithm,
    MotionConstraintBuildFunction motion_constraint_builder, const World &new_world)
{
    std::vector<std::unique_ptr<Intent>> intents;
    PriorityTacticVector priority_tactics;
    unsigned int num_tactics =
        static_cast<unsigned int>(new_world.friendlyTeam().numRobots());
    if (requires_goalie && new_world.friendlyTeam().goalie())
    {
        num_tactics--;
    }
    updateTactics(PlayUpdate(new_world, num_tactics,
                             [&priority_tactics](PriorityTacticVector new_tactics) {
                                 priority_tactics = std::move(new_tactics);
                             }));

    ConstPriorityTacticVector const_priority_tactics;

    // convert pointers to const pointers
    std::for_each(priority_tactics.begin(), priority_tactics.end(), [&](auto &tactics) {
        ConstTacticVector const_tactics = {};
        std::transform(tactics.begin(), tactics.end(), std::back_inserter(const_tactics),
                       [](std::shared_ptr<Tactic> tactic) { return tactic; });
        const_priority_tactics.push_back(const_tactics);
    });

    auto robot_tactic_assignment = robot_to_tactic_assignment_algorithm(
        const_priority_tactics, new_world, requires_goalie);

    for (auto tactic_vec : priority_tactics)
    {
        for (auto tactic : tactic_vec)
        {
            auto iter = robot_tactic_assignment.find(tactic);
            if (iter != robot_tactic_assignment.end())
            {
                auto intent = tactic->get(iter->second, new_world);
                intent->setMotionConstraints(motion_constraint_builder(*tactic));
                intents.push_back(std::move(intent));
            }
        }
    }
    return intents;
}

void Play::getNextTacticsWrapper(TacticCoroutine::push_type &yield)
{
    // Yield an empty vector the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield({});

    // The getNextTactics function is given the World as a parameter rather than using
    // the member variable since it's more explicit and obvious where the World
    // comes from when implementing Plays. The World is passed as a reference, so when
    // the world member variable is updated the implemented Plays will have access
    // to the updated world as well.
    if (world)
    {
        getNextTactics(yield, world.value());
    }
}

// TODO (#2359): delete once all plays are not coroutines
void Play::updateTactics(const PlayUpdate &play_update)
{
    play_update.set_tactics(getTactics(play_update.world));
}
