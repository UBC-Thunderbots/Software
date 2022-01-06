#pragma once

#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>
#include <vector>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

using RobotToTacticAssignmentFunction =
    std::function<std::map<std::shared_ptr<const Tactic>, Robot>(
        const ConstPriorityTacticVector&, const World&, bool)>;

using MotionConstraintBuildFunction =
    std::function<std::set<MotionConstraint>(const Tactic& tactic)>;

// This coroutine returns a list of list of shared_ptrs to Tactic objects
using TacticCoroutine = boost::coroutines2::coroutine<PriorityTacticVector>;

/**
 * In the STP framework, a Play is a collection of tactics that represent some
 * "team-wide" goal. It can be thought of like a traditional play in soccer.
 * Some examples are:
 * - A defense play
 * - An offense play
 * - A corner-kick of free-kick play
 *
 * Plays are stateful, and use Tactics to implement their behaviour.
 * For the most part, this statefulness is used to created "stages" for each Play.
 * For example, a corner-kick play be broken down into 2 stages:
 * - First, get the robots into position and find a good robot to pass to
 * - Execute the pass and attempt a one-touch shot on net
 *
 * These stages are useful in order for us to create more complex behaviour and write
 * it in a more understandable way.
 *
 * Plays define what conditions must be met for them to start (with the isApplicable
 * function), and what conditions must be continuously met for the Play to continue
 * running (with the invariantHolds function). These are very important to get right,
 * so that we can always run at least 1 Play in every scenario, and that Plays don't
 * unexpectedly stop during gameplay. See the documentation in stp.h for more info.
 */
class Play
{
   public:
    /**
     * Creates a new Play
     *
     * @param play_config The Play configuration
     * @param min_tactics The minimum number of tactics this play supports
     * @param requires_goalie Whether this plays requires a goalie
     */
    explicit Play(std::shared_ptr<const PlayConfig> play_config, unsigned int min_tactics,
                  bool requires_goalie);

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
     * Returns true if the Play is done and false otherwise. The Play is considered
     * done when its coroutine is done (the getNextTactics() function has no
     * more work to do)
     *
     * TODO (#2359): make pure virtual once all plays are not coroutines
     *
     * @return true if the Play is done and false otherwise
     */
    virtual bool done() const;

    /**
     * Gets the minimum number of tactics that the play supports
     *
     * @return the minimum number of tactics that the play supports
     */
    unsigned int minTactics();

    /**
     * Gets Intents from the Play given the assignment algorithm and world
     *
     * @param robot_to_tactic_assignment_algorithm The algorithm for assigning robots to
     * tactics
     * @param motion_constraint_builder Builds motion constraints from tactics
     * @param world The updated world
     *
     * @return the vector of intents to execute
     */
    std::vector<std::unique_ptr<Intent>> get(
        RobotToTacticAssignmentFunction robot_to_tactic_assignment_algorithm,
        MotionConstraintBuildFunction motion_constraint_builder, const World& new_world);

    virtual ~Play() = default;

   protected:
    // TODO (#2359): remove this
    // The Play configuration
    std::shared_ptr<const PlayConfig> play_config;

   private:
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
     * TODO (#2359): delete once all plays are not coroutines
     *
     * @param world The current state of the world
     * @return A list of shared_ptrs to the Tactics the Play wants to run at this time, in
     * order of priority
     */
    PriorityTacticVector getTactics(const World& world);

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
     * TODO (#2359): delete once all plays are not coroutines
     *
     * @param yield The coroutine push_type for the Play
     */
    void getNextTacticsWrapper(TacticCoroutine::push_type& yield);

    /**
     * This function yields a list of shared_ptrs to the Tactics the Play wants to run at
     * this time, in order of priority. This yield happens in place of a return.
     *
     * TODO (#2359): delete once all plays are not coroutines
     *
     * @param yield The coroutine push_type for the Play
     * @param world The current state of the world
     */
    virtual void getNextTactics(TacticCoroutine::push_type& yield,
                                const World& world) = 0;

    // TODO (#2359): make pure virtual once all plays are not coroutines
    virtual void updateTactics(const PlayUpdate& play_update);

    // Whether this plays requires a goalie
    const bool requires_goalie;

    // TODO (#2359): remove this
    // The coroutine that sequentially returns the Tactics the Play wants to run
    TacticCoroutine::pull_type tactic_sequence;

    // TODO (#2359): remove this
    // The Play's knowledge of the most up-to-date World
    std::optional<World> world;

    // TODO (#2359): remove this
    PriorityTacticVector priority_tactics;

    unsigned int min_tactics;
};
