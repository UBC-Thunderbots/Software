#pragma once

#include <boost/coroutine2/all.hpp>
#include <vector>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"
#include "software/ai/navigator/trajectory/trajectory_planner.h"

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
     * @param ai_config_ptr shared pointer to ai_config
     * @param requires_goalie Whether this plays requires a goalie
     */
    explicit Play(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr,
                  bool requires_goalie);

    /**
     * Gets Primitives from the Play given the the world, and inter-play communication
     *
     * @param world The updated world
     * @param inter_play_communication The inter-play communication struct
     * @param set_inter_play_communication_fun The callback to set the inter-play
     * communication struct
     *
     * @return the PrimitiveSet to execute
     */
    virtual std::unique_ptr<TbotsProto::PrimitiveSet> get(
        const WorldPtr& world_ptr, const InterPlayCommunication& inter_play_communication,
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

   protected:
    // A shared pointer to the ai configuration to configure ai behaviour, shared by all
    // Plays, Tactics, and FSMs
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr;

    // Goalie tactic common to all plays
    std::shared_ptr<GoalieTactic> goalie_tactic;

    std::map<std::shared_ptr<const Tactic>, RobotId> tactic_robot_id_assignment;

    // Cached robot trajectories
    std::map<RobotId, TrajectoryPath> robot_trajectories;

    // List of all obstacles in the world at the current iteration
    // and all robot paths. Used for visualization
    TbotsProto::ObstacleList obstacle_list;
    TbotsProto::PathVisualization path_visualization;

    // TODO (#2359): make pure virtual once all plays are not coroutines
    /**
     * Updates the priority tactic vector with new tactics
     *
     * @param play_update The PlayUpdate struct that contains all the information for
     * updating the tactics
     */
    virtual void updateTactics(const PlayUpdate& play_update)=0;

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
    std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
               std::map<std::shared_ptr<const Tactic>, RobotId>>
    assignTactics(const WorldPtr& world_ptr, TacticVector tactic_vector,
                  const std::vector<Robot>& robots_to_assign);


    // HaltTactics common to all plays for robots that don't have tactics assigned
    TacticVector halt_tactics;

    // Whether this play requires a goalie
    const bool requires_goalie;

    uint64_t sequence_number = 0;

    RobotNavigationObstacleFactory obstacle_factory;
};
