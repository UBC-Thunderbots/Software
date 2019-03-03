#include "ai/hl/stp/play/play.h"

Play::Play() : tactic_sequence(boost::bind(&Play::getNextTacticsWrapper, this, _1)) {}

bool Play::done() const
{
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done.
    return !static_cast<bool>(tactic_sequence);
}

std::optional<std::vector<std::shared_ptr<Tactic>>> Play::getTactics(const World &world)
{
    // Update the member variable that stores the world. This will be used by the
    // getNextTacticsWrapper function (inside the coroutine) to pass the World data to
    // the getNextTactics function. This is easier than directly passing the World data
    // into the coroutine
    this->world = world;
    if (tactic_sequence)
    {
        // Calculate and return the next Intent
        tactic_sequence();
        auto next_tactics = tactic_sequence.get();
        return std::make_optional(next_tactics);
    }
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done so wereturn an empty
    // optional
    return std::nullopt;
}

std::vector<std::shared_ptr<Tactic>> Play::getNextTacticsWrapper(
    TacticCoroutine::push_type &yield)
{
    // Yield an empty vector the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield({});

    // Anytime after the first function call, the getNextTactics function will be
    // used to perform the real logic.
    // The getNextTactics function is given the World as a parameter rather than using
    // the member variable since it's more explicit and obvious where the World
    // comes from when implementing Plays
    return getNextTactics(yield, this->world);
}
