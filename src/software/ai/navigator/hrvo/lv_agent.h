#pragma once

#include "software/ai/navigator/hrvo/lv_agent.h"
#include "software/ai/navigator/hrvo/path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class LVAgent {

public:
    /**
     * Constructor
     *
     * @prarm robotState            The robots current state
     * @param radius                The radius of this agent.
     * @param path                  The path of this agent
     * @param robot_id	            The robot id for this agent.
     * @param type	  	            The team side for this agent (friendly or enemy).
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     */

    /**
 * Computes the new velocity of this agent.
 * @param agents is unused
 */
    void computeNewVelocity(std::vector<Agent> &agents, double time_step) override;

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;

protected:

    RobotState robotState;
    // This agent's current actual radius
    float radius_;
    // The path of this Agent
    AgentPath path;
    // robot id of this Agent
    RobotId robot_id;
    // whether this Agent is FRIENDLY or ENEMY
    TeamSide agent_type;

    // remove depending on simulator impl
    // The minimum radius which this agent can be
    const float min_radius_;
    // The maximum amount which the radius can increase by
    const float max_radius_inflation_;
    const float max_speed_;
    const float max_accel_;

};