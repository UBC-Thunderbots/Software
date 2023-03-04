#pragma once

#include <map>

#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/world.h"
#include "proto/primitive.pb.h"


class Agent {
public:
    /**
     * Constructor
     *
     * @param robot_id	            The robot id for this agent.
     * @prarm robot_state           The robots current state
     * @param side	  	            The team side for this agent (friendly or enemy).
     * @param path                  The path of this agent
     * @param radius                The radius of this agent.
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can inflate.
     */
    Agent(RobotId robot_id, RobotState robot_state, TeamSide side, RobotPath &path,
          double radius, double max_speed, double max_accel, double max_radius_inflation);


    /**
     * Update the agent's path and current state
     *
     * @param time_step the current time step that the simulator is running at
     */
    void update(double time_step);


    /**
     * Update the agent's radius based on its current (robot_state) velocity
     */
    void updateRadiusFromVelocity();


    /**
     * Update the primitive which this agent is currently pursuing.
     *
     * @param new_primitive The new primitive to pursue
     * @param world The world in which the new primitive is being pursued
     * @param time_step the time_step to use to step at
     */
    virtual void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                 const World &world,
                                 double time_step) = 0;

    /**
     * Computes the preferred velocity of this agent.
     *
     * @param time_step
     * @return the computed preferred velocity
     */
    virtual Vector computePreferredVelocity(double time_step) = 0;

    /**
     * Run the HRVO algorithm, and compute the new velocity this agent should move at
     *
     * @param robots robots in the simulation
     * @param time_step
     */
    virtual void computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots, double time_step) = 0;


    /**
     * Create the VO to represent the given agent, relative to this agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;


    /**
     * Return the state of the agent
     *
     * @return The RobotState for this agent
     */
    RobotState getRobotState();

    /**
     * Get the preferred velocity for this robot
     *
     * @return the computed preferred velocity, if there is one.
     */
    Vector getPreferredVelocity() const;

    /**
     * Gets the max speed for this agent
     *
     * @return max speed for this agent
     */
    double getMaxSpeed() const;

    /**
     * Gets the radius for this agent
     *
     * @return radius for this agent
     */
    double getRadius() const;

    /**
     * Gets the the path for this agent
     *
     * @return Path for this agent
     */
    RobotPath &getPath();

    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    double getMaxAccel() const;

    /**
     * Set this robots preferred velocity
     *
     * @param The velocity for which preferred_velocity should be set to
     */
    void setPreferredVelocity(Vector velocity);

    // robot id of this Agent
    RobotId robot_id;
    // current state
    RobotState robot_state;
    // whether this Agent is FRIENDLY or ENEMY
    TeamSide side;
    // The path of this Agent
    RobotPath path;

    // This agent's current actual radius
    double radius;
    // The minimum radius which this agent can be
    const double min_radius;

    double max_speed;
    const double max_accel;
    // The maximum amount which the radius can increase by

    const double max_radius_inflation;

    Vector new_velocity;

protected:
    Vector preferred_velocity;
};


