#pragma once

#include "agent.h"
#include "simulator.h"
#include "vector2.h"

/**
 * An agent/robot in the simulation which has linear velocity.
 */
class LinearVelocityAgent : public Agent
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
    LinearVelocityAgent(Simulator *simulator, const Vector2 &position, float radius, const Vector2 &velocity,
                        float maxSpeed, float maxAccel, std::size_t goalNo,
                        float goalRadius);

    virtual ~LinearVelocityAgent() override = default;

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity() override;
};
