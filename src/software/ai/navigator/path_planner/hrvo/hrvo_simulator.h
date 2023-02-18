#pragma once

#include <fstream>
#include <limits>
#include <vector>

#include "software/ai/navigator/path_planner/hrvo/hrvo_agent.h"
#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/nearest_neighbor_search.hpp"

class HRVOSimulator {
public:
    /**
     * Constructor
     * @param time_step The time step between each step of the simulator
     * @param robot_constants The robot constants to be used for all Agents representing a
     * robot
     * @param friendly_team_colour The colour of the friendly team
     */
    explicit HRVOSimulator(double time_step, const RobotConstants_t &robot_constants,
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

    void doStep(Duration time_step);

    /**
     * Assumption: friendly team is the only robots running HRVO,
     * only enemy robots have robot id offset
     * @param robot_id
     */
    void visualize(RobotId robot_id);

private:

    /**
     *      Adds a new Hybrid Reciprocal Agent to the simulation based on Robot.
     *
     * @param robot    The robot which this agent should be based on
     * @param type     Whether this robot is FRIENDLY or ENEMY
     *
     */
    void configureHRVORobot(const Robot &robot);

    /**
     *      Adds a new Linear Velocity Agent to the simulation based on Robot.
     *
     * @param robot       	The robot which this agent should be based on
     * @param destination 	Destination for this robot
     * @param type 		Whether this robot is FRIENDLY or ENEMY
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
    Duration time_step;


    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr double FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05;
    static constexpr double ENEMY_ROBOT_RADIUS_MAX_INFLATION = 0.06;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    static constexpr double GOAL_RADIUS_SCALE = 1.05;

    const unsigned int ENEMY_LV_ROBOT_OFFSET = 1000;
};
