#pragma once

#include <fstream>
#include <limits>
#include <vector>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"


class HRVOSimulatorNew {
public:
    /**
     * Constructor
     * @param time_step The time step between each step of the simulator
     * @param robot_constants The robot constants to be used for all Agents representing a
     * robot
     * @param friendly_team_colour The colour of the friendly team
     */
    explicit HRVOSimulatorNew(float time_step, const RobotConstants_t &robot_constants,
                           const TeamColour friendly_team_colour);

    /**
     * Reset all agents to match the state of the given world.
     * Friendly robots will use the Hybrid Reciprocal algorithm to traverse.
     * Enemy robots will go directly towards their goal without trying to avoid any
     * obstacles
     *
     * @param world The world which the simulation should be based upon
     */
    void updateWorld(const World &world);

    /**
     * Reset all friendly agents goal points to match the path of the given primitive set
     *
     * @param new_primitive_set
     */
    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set);

    // comment
    void doStep();
    void computeNeighbors(const HRVOAgent &agent);

private:

    /**
     *      Adds a new Hybrid Reciprocal Agent to the simulation based on Robot.
     *
     * @param robot    The robot which this agent should be based on
     * @param type     Whether this robot is FRIENDLY or ENEMY
     *
     * @return    The index of the agent.
     */
    std::size_t addHRVORobotAgent(const Robot &robot, TeamSide type);

    /**
     *      Adds a new Linear Velocity Agent to the simulation based on Robot.
     *
     * @param robot       	The robot which this agent should be based on
     * @param destination 	Destination for this robot
     * @param type 		Whether this robot is FRIENDLY or ENEMY
     * @return    The index of the agent.
     */
    std::size_t addLinearVelocityRobotAgent(const Robot &robot, const Vector &destination,
                                            TeamSide type);

    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set;

    // Latest World which the simulator has received
    std::optional<World> world;

    // The robot constants which all agents will use
    RobotConstants_t robot_constants;

    // The colour of the friendly team
    const TeamColour friendly_team_colour;
    // will be removed
    // KdTree used to calculate the K nearest agents
    std::unique_ptr<KdTree> kd_tree;

    // List of agents (robots) in this simulation
    std::vector<Agent> agents;

    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr float FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05f;
    static constexpr float ENEMY_ROBOT_RADIUS_MAX_INFLATION    = 0.06f;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    static constexpr float GOAL_RADIUS_SCALE = 1.05f;

    // How much larger should the goal radius be (in meters). This is added as a safety
    // tolerance so robots do not accidentally enter the minimum distance threshold.
    // NOTE: This value must be >= 0
    static constexpr float BALL_AGENT_RADIUS_OFFSET = 0.1f;

    // The scale multiple of max robot speed which the preferred speed will be set at.
    // pref_speed = max_speed * PREF_SPEED_SCALE
    // NOTE: This scale multiple must be <= 1
    static constexpr float PREF_SPEED_SCALE = 0.85f;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    static constexpr float MAX_NEIGHBOR_SEARCH_DIST = 2.5f;
    // The maximum number of neighbors/agents to consider when drawing velocity obstacles.
    static constexpr unsigned int MAX_NEIGHBORS = 15;
};