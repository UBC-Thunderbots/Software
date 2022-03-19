#pragma once

#include "extlibs/hrvo/path.h"
#include "software/geom/vector.h"

class Simulator;

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
     * @peram path               The path for this agent
     * @param goalRadius         The goal radius of this agent.
     */
    Agent(Simulator *simulator, const Vector &position, float radius,
          const Vector &velocity, const Vector &prefVelocity, float maxSpeed,
          float maxAccel, Path &path);

    virtual ~Agent() = default;

    /**
     * A hybrid reciprocal velocity obstacle.
     */
    class VelocityObstacle
    {
       public:
        VelocityObstacle() = default;

        // The position of the apex of the hybrid reciprocal velocity obstacle.
        Vector apex_;

        // The direction of the first side of the hybrid reciprocal velocity obstacle.
        Vector side1_;

        // The direction of the second side of the hybrid reciprocal velocity obstacle.
        Vector side2_;
    };

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
    const Path &getPath() const;

   protected:
    // Agent Properties
    Vector position_;
    float radius_;

    // The actual current velocity of this Agent
    Vector velocity_;
    // The requested new velocity of this Agent
    Vector new_velocity_;
    // The desired new velocity of this Agent
    Vector pref_velocity_;
    // The path of this Agent
    Path path;

    float max_speed_;
    float max_accel_;
    bool reached_goal_;

    Simulator *const simulator_;
};
