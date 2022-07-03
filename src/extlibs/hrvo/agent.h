#pragma once

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"

class HRVOSimulator;

enum AgentType
{
    FRIENDLY,
    ENEMY,
    BALL
};

/**
 * An agent/robot in the HRVO simulation.
 */
class Agent
{
   public:
    /**
     * Constructor
     *
     * @param simulator          The simulator which this agent is a part of
     * @param position           The starting position of this agent.
     * @param radius             The radius of this agent.
     * @param velocity           The initial velocity of this agent.
     * @param prefVelocity       The preferred velocity of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param path               The path for this agent
     */
    Agent(HRVOSimulator *simulator, const Vector &position, float radius,
          const Vector &velocity, const Vector &prefVelocity, float maxSpeed,
          float maxAccel, AgentPath &path, RobotId robot_id, AgentType agent_type);

    virtual ~Agent() = default;

    /**
     * Computes the new velocity of this agent.
     */
    virtual void computeNewVelocity() = 0;

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;

    /**
     * Updates the position and velocity of this agent.
     */
    virtual void update();

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

    RobotId getRobotId();

    AgentType getAgentType();

   protected:
    // Agent Properties
    Vector position_;
    float radius_;

    // The actual current velocity of this Agent
    Vector velocity_;
    // The requested new velocity of this Agent
    Vector new_velocity_;
    // The desired new speed of this Agent
    // NOTE: HRVO algorithm will try to pick this speed, however, it may pick a different
    // speed to avoid collisions.
    Vector pref_velocity_;
    // The path of this Agent
    AgentPath path;

    RobotId robot_id;

    float max_speed_;
    float max_accel_;
    bool reached_goal_;

    AgentType agent_type;

    HRVOSimulator *const simulator_;
};
