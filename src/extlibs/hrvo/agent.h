#pragma once

#include "software/geom/vector.h"

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
     * @param simulator          The simulator which this agent is a part of
     * @param position           The starting position of this agent.
     * @param radius             The radius of this agent.
     * @param velocity           The initial velocity of this agent.
     * @param prefVelocity       The preferred velocity of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param goalIndex          The index of the Goal which this agent should go to.
     * @param goalRadius         The goal radius of this agent.
     */
    Agent(HRVOSimulator *simulator, const Vector &position, float radius,
          const Vector &velocity, const Vector &prefVelocity, float maxSpeed,
          float maxAccel, std::size_t goalIndex, float goalRadius);

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
    float getGoalRadius() const;

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

    float max_speed_;
    float max_accel_;

    std::size_t goal_index_;
    float goal_radius_;
    bool reached_goal_;

    // TODO (#2373): Remove once new Path class is added and add timeStep as a argument to
    // update(time_step)
    HRVOSimulator *const simulator_;
};
