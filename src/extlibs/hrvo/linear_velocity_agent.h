#pragma once

#include "agent.h"
#include "simulator.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"

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
     * @param simulator   The simulation which this agent runs in.
     * @param position              The starting position of this agent.
     * @param radius                The radius of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     * @param velocity              The initial velocity of this agent.
     * @param maxSpeed              The maximum speed of this agent.
     * @param maxAccel              The maximum acceleration of this agent.
     * @param path                  The path of this agent
     * @param robot_id	  The robot id for this agent.
     * @param type	  	  The team side for this agent (friendly or enemy).
     */
    LinearVelocityAgent(HRVOSimulator *simulator, const Vector &position, float radius,
                        float max_radius_inflation, const Vector &velocity,
                        float maxSpeed, float maxAccel, AgentPath &path, RobotId robot_id,
                        TeamSide type);

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
