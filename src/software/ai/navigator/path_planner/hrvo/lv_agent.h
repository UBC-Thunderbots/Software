#pragma once

#include "software/ai/navigator/path_planner/hrvo/agent.h"
#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "software/ai/navigator/path_planner/hrvo/hrvo_agent.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class LVAgent : public Agent {

public:
    /**
     * Constructor
     *
     * @prarm robot_state            The robots current state
     * @param radius                The radius of this agent.
     * @param path                  The path of this agent
     * @param robot_id	            The robot id for this agent.
     * @param type	  	            The team side for this agent (friendly or enemy).
     * inflate.
     */

    LVAgent(RobotId robot_id, RobotState robot_state, TeamSide side, RobotPath &path,
            double radius, double max_speed, double max_accel, double max_radius_inflation);

    /**
     * Computes the new velocity of this agent.
     * @param agents is unused
     */
    void computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots, Duration time_step) override;

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;

    Vector computePreferredVelocity(Duration time_step) override;

    void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                 const World &world,
                                 Duration time_step) override;

};