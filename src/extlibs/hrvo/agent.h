#pragma once

#include "extlibs/hrvo/vector2.h"
#include "extlibs/hrvo/simulator.h"

/**
 * An agent/robot in the HRVO simulation.
 */
class Agent
{
   public:
    /**
     * Constructor
     *
     * @param simulator  The simulator which this agent is a part of
     */
    explicit Agent(Simulator *simulator); // TODO: Might be able to remove this constructor as the init values dont really make sense

    /**
     * Constructor
     *
     * @param simulator          The simulator which this agent is a part of
     * @param position           The starting position of this agent.
     * @param radius             The radius of this agent.
     * @param velocity           The initial velocity of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param goalNo             The goal number of this agent.
     * @param goalRadius         The goal radius of this agent.
     */
    Agent(Simulator *simulator, const Vector2 &position, float radius,
          const Vector2 &velocity, float maxSpeed, float maxAccel, std::size_t goalNo,
          float goalRadius);

    virtual ~Agent() = 0;

    /**
     * Computes the new velocity of this agent.
     */
    virtual void computeNewVelocity() = 0;

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

    // protected: TODO: make properties protected
    // Agent Properties
    Vector2 position_;
    float radius_;

    Vector2 velocity_;
//    Vector2 prefVelocity_; // TODO: Add this
    Vector2 newVelocity_;
    float maxSpeed_;
    float maxAccel_;

    std::size_t goalNo_;
    float goalRadius_;
    bool reachedGoal_;

    // Used to get the simulator time step
    // TODO: Remove once new Path class is added and add timeStep as a argument to
    // update(...)
    Simulator *const simulator_;

    friend class KdTree;  // TODO: Ideally we use getters instead of friending the class
    friend class Simulator;
};
