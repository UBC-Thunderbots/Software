#pragma once

#include <boost/coroutine2/all.hpp>
#include <memory>
#include <vector>

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/world/world.h"

// We typedef the coroutine return type to make it shorter, more descriptive,
// and easier to work with.
// This coroutine returns a list of shared_ptrs to Tactic objects
typedef boost::coroutines2::coroutine<std::vector<std::shared_ptr<Tactic>>>
    TacticCoroutine;

/**
 * In the STP framework, a Play is a collection of tactics that represent some
 * "team-wide" goal. It can be thought of like a traditional play in soccer, such as an
 * offensive play, defensive play, or a specific play used for corner kicks.
 *
 * When we are running autonomously, different Plays are selected at different times
 * based on the state of the world, and this switching of Plays is what allows our AI to
 * play a full game of soccer, as long a we provide a Play for any given scenario.
 *
 * Plays must define what conditions must be met for them to start (with the isApplicable
 * function), and what conditions must be continously met for the Play to continue
 * running (with the invariantHolds function). These are very important to get right,
 * so that we can always run at least 1 Play in every scenario, and that Plays don't
 * unexpectedly stop.
 */
class Play
{
   public:
    /**
     * Creates a new Play
     */
    explicit Play();

    /**
     * Returns whether or not this Play can be started. For example, the Enemy Team
     * must have the ball for a defensive play to be applicable.
     *
     * @param world The current state of the world
     * @return true if this Play can be started, and false otherwise
     */
    virtual bool isApplicable(const World& world) const = 0;

    /**
     * Returns whether or not the invariant for this Play holds (is true). The invariant
     * is a set of conditions that must remain true for the Play to continue running
     * (not necessarily the same as the Applicable conditions). For example, the Invariant
     * for an Offensive Play might be that our team must have possession of the ball. If
     * our team no longer has possession of the ball (ie. the invariant no longer holds),
     * then we should probably run a different Play.
     *
     * @param world The current state of the world
     * @return true if this Play's invariant holds (is true), and false otherwise
     */
    virtual bool invariantHolds(const World& world) const = 0;

    /**
     * Returns a list of shared_ptrs to the Tactics the Play wants to run at this time, in
     * order of priority. The Tactic at the beginning of the vector has the highest
     * priority, and the Tactic at the end has the lowest priority.
     *
     * shared_ptrs are used so that the Play can own the objects (and have control over
     * updating the Tactic parameters, etc), but callers of this function can still
     * access their updated state. Using unique_ptrs wouldn't allow the Play to maintain
     * the Tactic's state because the objects would have to be constructed and moved every
     * time the function is called.
     *
     * @param world The current state of the world
     * @return A list of shared_ptrs to the Tactics the Play wants to run at this time, in
     * order of priority
     */
    std::optional<std::vector<std::shared_ptr<Tactic>>> getTactics(const World& world);

    /**
     * Returns true if the Play is done and false otherwise. The Play is considered
     * done when its coroutine is done (the getNextTactics() function has no
     * more work to do)
     *
     * @return true if the Play is done and false otherwise
     */
    bool done() const;

    /**
     * Returns the name of this Play
     *
     * @return the name of this Play
     */
    virtual std::string getName() const = 0;

    virtual ~Play() = default;

   private:
    /**
     * A wrapper function for the getNextTactics function.
     *
     * This function exists because when the coroutine (tactic_sequence) is first
     * constructed the coroutine is called/entered. This would normally cause the
     * getNextTactics to be run once and potentially return incorrect results
     * due to default constructed values.
     *
     * This wrapper function will yield an empty vector the first time it's called and
     * otherwise use the getNextTactics function. This first "empty" value will never
     * be seen/used by the rest of the system since this will be during construction,
     * and the coroutine will be called again with valid parameters before any values are
     * returned. This effectively "shields" the logic from any errors caused by default
     * values during construction.
     *
     * This function yields a list of shared_ptrs to the Tactics the Play wants to run at
     * this time, in order of priority. This yield happens in place of a return.
     *
     * @param yield The coroutine push_type for the Play
     */
    void getNextTacticsWrapper(TacticCoroutine::push_type& yield);

    /**
     * This function yields a list of shared_ptrs to the Tactics the Play wants to run at
     * this time, in order of priority. This yield happens in place of a return.
     *
     * @param yield The coroutine push_type for the Play
     * @param world The current state of the world
     */
    virtual void getNextTactics(TacticCoroutine::push_type& yield,
                                const World& world) = 0;

    // The coroutine that sequentially returns the Tactics the Play wants to run
    TacticCoroutine::pull_type tactic_sequence;

    // The Play's knowledge of the most up-to-date World
    std::optional<World> world;
};
