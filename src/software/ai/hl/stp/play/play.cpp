#include "software/ai/hl/stp/play/play.h"

Play::Play() : tactic_sequence(boost::bind(&Play::getNextTacticsWrapper, this, _1)) {}

bool Play::done() const
{
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done.
    return !static_cast<bool>(tactic_sequence);
}

std::vector<std::shared_ptr<Tactic>> Play::getTactics(const World &world)
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
    }
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done so we return an empty
    // vector
    return std::vector<std::shared_ptr<Tactic>>();
}

void Play::getNextTacticsWrapper(TacticCoroutine::push_type &yield)
{
    // Yield an empty vector the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield({});

    // Anytime after the first function call, the getNextTactics function will be
    // used to perform the real logic. The calculateNextIntent function will yield its
    // values to the top of the coroutine stack, where they will be retrieved by
    // getNextAction, so we do not need to yield or return the result of this function
    //
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
