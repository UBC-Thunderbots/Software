#pragma once

#include <boost/bind.hpp>
#include <boost/coroutine2/all.hpp>
#include <vector>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play_fsm.h"
#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/navigator/path_planner/global_path_planner_factory.h"

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
    /**
     * Creates a new Play
     *
     * @param ai_config The AI configuration
     * @param requires_goalie Whether this plays requires a goalie
     * @param strategy   to get and store shared calculations
     */
    explicit Play(TbotsProto::AiConfig ai_config, bool requires_goalie,
                  std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());

    /**
     * Resets the play, required after a change in the AiConfig.
     */
    virtual void reset();

    /**
     * Gets Primitives from the Play given the path planner factory, the world, and
     * inter-play communication
     *
     * @param path_planner_factory The path planner factory
     * @param world The updated world
     * @param inter_play_communication The inter-play communication struct
     * @param set_inter_play_communication_fun The callback to set the inter-play
     * communication struct
     *
     * @return the PrimitiveSet to execute
     */
    virtual std::unique_ptr<TbotsProto::PrimitiveSet> get(
        const GlobalPathPlannerFactory& path_planner_factory, const World& world,
        const InterPlayCommunication& inter_play_communication,
        const SetInterPlayCommunicationCallback& set_inter_play_communication_fun);

    /**
     * Get tactic to robot id assignment
     *
     * @return a map from tactic to robot id
     */
    const std::map<std::shared_ptr<const Tactic>, RobotId>& getTacticRobotIdAssignment()
        const;

    virtual ~Play() = default;

    /**
     * Gets the state of the Play
     *
     * @return a vector strings representing the state
     */
    virtual std::vector<std::string> getState();

    /**
     * Updates this Play's AI config in case of a parameter change.
     *
     * @warning this function is NOT thread-safe
     *
     * @param new_config the new AI config to use for this Play
     */
    void updateAiConfig(const TbotsProto::AiConfig& new_config);

   protected:
    // The Play configuration
    TbotsProto::AiConfig ai_config;

    std::shared_ptr<Strategy>
        strategy;  // holds information about coordinating strategy between multiple Plays

    // Goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;

    std::map<std::shared_ptr<const Tactic>, RobotId> tactic_robot_id_assignment;

    // TODO (#2359): make pure virtual once all plays are not coroutines
    /**
     * Updates the priority tactic vector with new tactics
     *
     * @param play_update The PlayUpdate struct that contains all the information for
     * updating the tactics
     */
    virtual void updateTactics(const PlayUpdate& play_update);

    inline virtual void initialize(){};

    /**
     * Gets Primitives from a Tactic given the path planner factory, the world, and the
     * tactic
     *
     * @param path_planner_factory The path planner factory
     * @param world The updated world
     * @param tactic the Tactic
     * @param motion_constraints the motion constraints to use
     *
     * @return the PrimitiveSet to execute
     */
    static std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitivesFromTactic(
        const GlobalPathPlannerFactory& path_planner_factory, const World& world,
        std::shared_ptr<Tactic> tactic,
        std::set<TbotsProto::MotionConstraint> motion_constraints);

   private:
    /**
     * Assigns the given tactics to as many of the given robots
     *
     * @param path_planner_factory The path planner factory
     * @param world The world
     * @param tactic_vector The tactic vector
     * @param robots_to_assign The robots to assign to
     *
     * @return the remaining unassigned robots, the new primitives to assign, and robot to
     * tactic assignment
     */
    static std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
                      std::map<std::shared_ptr<const Tactic>, RobotId>>
    assignTactics(const GlobalPathPlannerFactory& path_planner_factory,
                  const World& world, TacticVector tactic_vector,
                  const std::vector<Robot> robots_to_assign);

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
    std::optional<World> world_;

    uint64_t sequence_number = 0;

    bool should_reset;
};
