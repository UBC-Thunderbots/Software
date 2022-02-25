#pragma once

#include "extlibs/hrvo/vector2.h"

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
    Agent(HRVOSimulator *simulator, const Vector2 &position, float radius,
          const Vector2 &velocity, const Vector2 &prefVelocity, float maxSpeed,
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
        Vector2 apex_;

        // The direction of the first side of the hybrid reciprocal velocity obstacle.
        Vector2 side1_;

        // The direction of the second side of the hybrid reciprocal velocity obstacle.
        Vector2 side2_;
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
    const Vector2 &getPosition() const;

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
    const Vector2 &getVelocity() const;

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
    const Vector2 &getPrefVelocity() const;

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
    void setPosition(const Vector2 &position);

    /**
     * Update the velocity of Agent
     * @param velocity New velocity
     */
    void setVelocity(const Vector2 &velocity);

    /**
     * Update the max speed of Agent
     * @param max_speed New max speed
     */
    void setMaxSpeed(float max_speed);

   protected:
    // Agent Properties
    Vector2 position_;
    float radius_;

    // The actual current velocity of this Agent
    Vector2 velocity_;
    // The requested new velocity of this Agent
    Vector2 new_velocity_;
    // The desired new velocity of this Agent
    Vector2 pref_velocity_;

    float max_speed_;
    float max_accel_;

    std::size_t goal_index_;
    float goal_radius_;
    bool reached_goal_;

    // TODO (#2373): Remove once new Path class is added and add timeStep as a argument to
    // update(time_step)
    HRVOSimulator *const simulator_;
};
