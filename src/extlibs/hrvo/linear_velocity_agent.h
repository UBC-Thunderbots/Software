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
     * @param simulator     The simulator which this agent is a part of.
     * @param position    The starting position of this agent.
     * @param velocity    The initial velocity of this agent.
     * @param max_speed    The maximum speed of this agent.
     * @param max_accel    The maximum acceleration of this agent.
     * @param path        The path of this agent
     * @param radius      The radius of this agent.
     */
    LinearVelocityAgent(HRVOSimulator *simulator, const Vector &position,
                        const Vector &velocity, float max_speed, float max_accel,
                        AgentPath &path, float radius);

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
    Agent::VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;
};
