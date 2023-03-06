#pragma once

#include <fstream>
#include <limits>
#include <vector>

#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/ai/navigator/path_planner/hrvo/hrvo_agent.h"
#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "software/ai/navigator/path_planner/hrvo/path_point.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/nearest_neighbor_search.hpp"
#include "software/geom/vector.h"
#include "software/world/world.h"

class HRVOSimulator
{
   public:
    /**
     * Constructor
     * @param time_step The time step between each step of the simulator
     * @param robot_constants The robot constants to be used for all Agents representing a
     * robot
     * @param friendly_team_colour The colour of the friendly team
     */
    explicit HRVOSimulator(double time_step, const RobotConstants_t &robot_constants,
                           TeamColour friendly_team_colour);


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


    /**
     * Performs a simulation step; updates the position, and velocity
     * of each agent, and the progress of each towards its goal by moving
     * the simulation time_step seconds forward
     */
    void doStep();


    /**
     * Get the current friendly robot velocity
     *
     * @param robot_id The robot id of the friendly robot to retrieve velocity from
     * @return Current global velocity of robot
     */
    Vector getRobotVelocity(unsigned int robot_id) const;


    /**
     * Update the velocity of the agent with the given id
     * @param robot_id Robot id of the agent to update
     * @param new_velocity New global velocity of the agent
     */
    void updateRobotVelocity(RobotId robot_id, const Vector &new_velocity);


    /**
     *  Returns the count of agents in the simulation.
     *
     * @return The count of agents in the simulation.
     */
    std::size_t getRobotCount();


    /**
     * Get all the robots running in this simulation
     *
     * @return a map of from robot id's to agents
     */
    std::map<RobotId, std::shared_ptr<Agent>> getRobots();


    /**
     * visualize the HRVO obstacles for friendly team
     *
     * @param robot_id
     */
    void visualize(RobotId robot_id);

   private:
    /**
     * Configure and add a HRVO Agent to the simulation.
     *
     * @param robot The robot for which this agent is based on
     */
    void configureHRVORobot(const Robot &robot);


    /**
     *  Configure and add a LV Agent to the simulation.
     *
     * @param robot The robot for which this agent is based on
     */
    void configureLVRobot(const Robot &robot);


    // The robot constants which all agents will use
    RobotConstants_t robot_constants;

    // robot id to agent
    std::map<RobotId, std::shared_ptr<Agent>> robots;

    // Latest World which the simulator has received
    std::optional<World> world;

    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set;


    // The colour of the friendly team
    const TeamColour friendly_team_colour;
    // simulator time step
    double time_step;

    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr double FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05;

    static constexpr double ENEMY_ROBOT_RADIUS_MAX_INFLATION = 0.06;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    static constexpr double GOAL_RADIUS_SCALE = 1.05;

    // index offset for enemy robot ids in the `robots` map
    const unsigned int ENEMY_LV_ROBOT_OFFSET = 1000;
};
