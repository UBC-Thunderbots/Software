#pragma once

#include "agent.h"
#include "simulator.h"
#include "software/geom/vector.h"

/**
 * An agent/robot in the simulation which has linear velocity.
 * This agent will go directly towards its destination, and has no sense of obstacles
 */
class LinearVelocityAgent : public Agent
{
   public:
    /**
     * Constructor
     *
     * @param position    The starting position of this agent.
     * @param radius      The radius of this agent.
     * @param velocity    The initial velocity of this agent.
     * @param maxSpeed    The maximum speed of this agent.
     * @param maxAccel    The maximum acceleration of this agent.
     * @param path        The path of this agent
     */
    LinearVelocityAgent(HRVOSimulator *simulator, const Vector &position, float radius,
                        const Vector &velocity, float maxSpeed, float maxAccel,
                        AgentPath &path);

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity() override;

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;
};
