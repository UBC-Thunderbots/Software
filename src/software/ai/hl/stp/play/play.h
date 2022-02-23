#pragma once

#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>
#include <vector>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
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
 */
class Play
{
   public:
    friend class STP;
    /**
     * Creates a new Play
     *
     * @param ai_config The Play configuration
     * @param requires_goalie Whether this plays requires a goalie
     */
    explicit Play(std::shared_ptr<const AiConfig> ai_config, bool requires_goalie);

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

    // TODO (#2359): make pure virtual once all plays are not coroutines
    /**
     * Updates the priority tactic vector with new tactics
     *
     * @param play_update The PlayUpdate struct that contains all the information for
     * updating the tactics
     */
    virtual void updateTactics(const PlayUpdate& play_update);

    /**
     * Given a vector of vector of tactics and the current World, assigns robots
     * from the friendly team to each tactic
     *
     * Some tactics may not be assigned a robot, depending on if there is a robot
     * capable of performing that tactic
     *
     * This will clear all assigned robots from all tactics
     *
     * The outer vector ranks the inner vector of tactics by priority. Tactics in
     * lower indexes of the outer vector will be assigned first. For example:
     *
     * {
     *      {crease_defender_1, crease_defender_2},
     *      {move_tactic},
     * }
     *
     * The cost of assigning both the crease_defender tactics will be minimized across
     * all robots first, followed by the move_tactic.
     *
     * The order of the given tactics in the inner vector also determines their priority,
     * with the tactics at the beginning of the vector being a higher priority than those
     * at the end. The priority determines which tactics will NOT be assigned if there are
     * not enough robots on the field to assign them all. For example, if a Play returned
     * 4 Tactics in total but there were only 3 robots on the field at the time, only the
     * first 3 Tactics in the vectors would be assigned to robots and run. (In the example
     * above, only the goalie and crease_defenders would be assigned)
     *
     * @param tactics The vector of vector of tactics that should be assigned a robot.
     * Note that this function modifies tactics to make the correct assignments, because
     * we need to modify the individual tactics _and_ possibly add/remove tactics
     * @param world The state of the world, which contains the friendly Robots that will
     * be assigned to each tactic
     *
     * @return map from assigned tactics to robot
     */
    std::unique_ptr<TbotsProto::PrimitiveSet> selectPrimitives(
        ConstPriorityTacticVector tactics, const World& world);

   protected:
    // TODO (#2359): remove this
    // The Play configuration
    std::shared_ptr<const AiConfig> ai_config;

    // Goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;

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

    // Stop tactic common to all plays for robots that don't have tactics assigned
    TacticVector stop_tactics;

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

    std::map<std::shared_ptr<const Tactic>, Robot> robot_tactic_assignment;
};

// Function that creates a play
using PlayConstructor =
    std::function<std::unique_ptr<Play>(std::shared_ptr<const AiConfig>)>;
