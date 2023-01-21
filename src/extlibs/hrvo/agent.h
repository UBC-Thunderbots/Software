#pragma once

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class HRVOSimulator;

/**
 * An agent/robot in the HRVO simulation.
 */
class Agent
{
   public:
    /**
     * Constructor
     *
     * @param position              The starting position of this agent.
     * @param radius                The radius of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     * @param velocity              The initial velocity of this agent.
     * @param pref_velocity          The preferred velocity of this agent.
     * @param max_speed              The maximum speed of this agent.
     * @param max_accel              The maximum acceleration of this agent.
     * @param path                  The path for this agent
     * @param robot_id		 		The robot id for this agent
     * @param agent_type	 		The friendly or enemy agent type
     */
    Agent(const Vector &position, float radius,
          float max_radius_inflation, const Vector &velocity, const Vector &prefVelocity,
          float maxSpeed, float maxAccel, AgentPath &path, RobotId robot_id,
          TeamSide agent_type);

    virtual ~Agent() = default;

    /**
     * Computes the new velocity of this agent.
     */
    virtual void computeNewVelocity(std::vector<std::shared_ptr<Agent>> &agents, double time_step) = 0;

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;

    /**
     * Update the agent's radius based on its current velocity
     */
    void updateRadiusFromVelocity();

    /**
     * Updates the position and velocity of this agent.
     * @param time_step the time step the simulator is currently using
     */
    virtual bool update(double time_step);

    /**
     * Returns the current position of the agent
     *
     * @return The current position of the agent
     */
    const Vector &getPosition() const;

    /**
     * Returns the agents radius
     *
     * @return The agents radius
     */
    float getRadius() const;

    /**
     * Return the current velocity of the agent
     *
     * @return The current velocity of the agent
     */
    const Vector &getVelocity() const;

    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    float getMaxAccel() const;

    /**
     * Return the goal radius of the agent
     *
     * @return The goal radius of the agent
     */
    float getPathRadius() const;

    /**
     * Return the preferred velocity of the agent
     *
     * @return The preferred velocity of the agent
     */
    const Vector &getPrefVelocity() const;

    /**
     * Return the Goal index for this agent
     *
     * @return The Goal index for this agent
     */
    size_t getGoalIndex() const;

    /**
     * Return true if this agent has reached its final goal, false otherwise.
     *
     * @return True if this agent has reached its final goal, false otherwise.
     */
    bool hasReachedGoal() const;

    /**
     * Gets the the path for this agent
     * @return Path for this agent
     */
    const AgentPath &getPath() const;

    /**
     * Gets the max speed for this agent
     * @return max speed for this agent
     */
    float getMaxSpeed() const;

    /**
     * Update position of Agent
     * @param position New position
     */
    void setPosition(const Vector &position);

    /**
     * Update radius of Agent
     * @param radius New radius
     */
    void setRadius(float radius);

    /**
     * Update the velocity of Agent
     * @param velocity New velocity
     */
    void setVelocity(const Vector &velocity);

    /**
     * Update the max speed of Agent
     * @param max_speed New max speed
     */
    void setMaxSpeed(float max_speed);

    /**
     * Sets a new path for this agent
     * @param new_path new path for this agent
     */
    void setPath(const AgentPath &new_path);

    /**
     * Gets the robot id for this Agent
     * @return robot id of the agent
     */
    RobotId getRobotId();

    /**
     * Get whether this robot is a FRIENDLY or an ENEMY robot.
     *
     * @return FRIENDLY if friendly, ENEMY if otherwise
     */
    TeamSide getAgentType();

   protected:
    // TODO use RobotState instead
    // Agent Properties
    Vector position_;
    // This agent's current actual radius
    float radius_;
    // The minimum radius which this agent can be
    const float min_radius_;
    // The maximum amount which the radius can increase by
    const float max_radius_inflation_;

    // The actual current velocity of this Agent
    Vector velocity_;
    // REMOVE
    // The requested new velocity of this Agent
    Vector new_velocity_;
    // REMOVE: Return these values from the associated func instead
    // The desired new speed of this Agent
    // NOTE: HRVO algorithm will try to pick this speed, however, it may pick a different
    // speed to avoid collisions.
    Vector pref_velocity_;
    // The path of this Agent
    AgentPath path;

    // robot id of this Agent
    RobotId robot_id;
    // whether this Agent is FRIENDLY or ENEMY
    TeamSide agent_type;
    // TODO private, const, underscored mfs
    //remove depending on simulator impl

    const float max_speed_;
    const float max_accel_;
};
