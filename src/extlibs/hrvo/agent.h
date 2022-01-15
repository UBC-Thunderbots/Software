#pragma once

#include "simulator.h"
#include "vector2.h"

/**
 * An agent/robot in the HRVO simulation.
 */
class Agent
{
   public:
    /**
     * Constructor
     *
     * @param position           The starting position of this agent.
     * @param radius             The radius of this agent.
     * @param velocity           The initial velocity of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param goalNo             The goal number of this agent.
     * @param goalRadius         The goal radius of this agent.
     */
    explicit Agent(Simulator *simulator) // TODO: Might be able to remove this constructor as the init values dont really make sense
            : simulator_(simulator),
              position_(Vector2(0.f, 0.f)),
              radius_(0.f),
              velocity_(Vector2(0.f, 0.f)),
              maxSpeed_(0.f),
              maxAccel_(0.f),
              goalNo_(0),
              goalRadius_(0.f),
              reachedGoal_(false)
    {
    }

    /**
     * Constructor
     *
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
          float goalRadius)
        : simulator_(simulator),
          position_(position),
          radius_(radius),
          velocity_(velocity),
          maxSpeed_(maxSpeed),
          maxAccel_(maxAccel),
          goalNo_(goalNo),
          goalRadius_(goalRadius),
          reachedGoal_(false)
    {
    }

    virtual ~Agent() = 0;

    /**
     * Computes the new velocity of this agent.
     */
    virtual void computeNewVelocity() = 0;

    /**
     * Updates the position and velocity of this agent.
     */
    virtual void update()
    {
        const float dv = abs(newVelocity_ - velocity_);

        if (dv < maxAccel_ * simulator_->timeStep_)
        {
            velocity_ = newVelocity_;
        }
        else
        {
            velocity_ = (1.0f - (maxAccel_ * simulator_->timeStep_ / dv)) * velocity_ +
                        (maxAccel_ * simulator_->timeStep_ / dv) * newVelocity_;
        }

        position_ += velocity_ * simulator_->timeStep_;

        if (absSq(simulator_->goals_[goalNo_]->getCurrentGoalPosition() - position_) <
            goalRadius_ * goalRadius_)
        {
            // Is at current goal position
            if (simulator_->goals_[goalNo_]->isGoingToFinalGoal())
            {
                reachedGoal_ = true;
            }
            else
            {
                simulator_->goals_[goalNo_]->getNextGoalPostion();
                reachedGoal_              = false;
                simulator_->reachedGoals_ = false;
            }
        }
        else
        {
            reachedGoal_              = false;
            simulator_->reachedGoals_ = false;
        }
    };

    /**
     * Returns the current position of the agent
     *
     * @return The current position of the agent
     */
    const Vector2 &getPosition() const
    {
        return position_;
    }

    /**
     * Returns the agents radius
     *
     * @return The agents radius
     */
    float getRadius() const
    {
        return radius_;
    }

    /**
     * Return the current velocity of the agent
     *
     * @return The current velocity of the agent
     */
    const Vector2 &getVelocity() const
    {
        return velocity_;
    }

    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    float getMaxAccel() const
    {
        return maxAccel_;
    }

    // protected: TODO: make properties protected
    // Agent Properties
    Vector2 position_;
    float radius_;

    Vector2 velocity_;
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
