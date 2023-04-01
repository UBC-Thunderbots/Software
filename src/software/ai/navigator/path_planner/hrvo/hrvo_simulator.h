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
     * @param robot_constants The robot constants to be used for all Agents representing a
     * robot
     * @param friendly_team_colour The colour of the friendly team
     */
    explicit HRVOSimulator();


    /**
     * Reset all agents to match the state of the given world.
     * Friendly robots will use the Hybrid Reciprocal algorithm to traverse.
     * Enemy robots will go directly towards their goal without trying to avoid any
     * obstacles
     *
     * @param world The world which the simulation should be based upon
     */
    void updateWorld(const World &world, const RobotConstants_t &robot_constants,
                     Duration time_step);


    /**
     * Reset all friendly agents goal points to match the path of the given primitive set
     *
     * @param new_primitive_set
     * @param time_step the time_step to use
     */
    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set,
                            Duration time_step);


    /**
     * Performs a simulation step; updates the position, and velocity
     * of each agent, and the progress of each towards its goal by moving
     * the simulation time_step seconds forward
     *
     * @param time_step the time_step to use
     */
    void doStep(Duration time_step);


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
    void visualize(RobotId robot_id, TeamColour friendly_team_colour);

    // Robot id offset for enemy robots so we don't have
    // friendly and enemy agents with overlapping ids in the `robots` map
    static const unsigned int ENEMY_LV_ROBOT_OFFSET = 1000;

   private:
    /**
     * Configure and add a HRVO Agent to the simulation.
     *
     * @param robot The robot for which this agent is based on
     * @param time_step the time_step to use
     */
    void configureHRVORobot(const Robot &robot, const RobotConstants_t &robot_constants,
                            Duration time_step);


    /**
     *  Configure and add a LV Agent to the simulation.
     *
     * @param robot The robot for which this agent is based on
     * @param time_step the time_step to use
     */
    void configureLVRobot(const Robot &robot, const RobotConstants_t &robot_constants,
                          Duration time_step);

    // robot id to agent
    std::map<RobotId, std::shared_ptr<Agent>> robots;

    // Latest World which the simulator has received
    std::optional<World> world;

    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set;

    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr double FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05;

    static constexpr double ENEMY_ROBOT_RADIUS_MAX_INFLATION = 0.06;
};
