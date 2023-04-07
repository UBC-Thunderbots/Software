#pragma once

#include "software/ai/navigator/path_planner/hrvo/agent.h"
#include "software/ai/navigator/path_planner/hrvo/hrvo_agent.h"
#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class LVAgent : public Agent
{
   public:
    /**
     * Constructor. This type of agent (Linear Velocity Agent)
     * moves directly towards its pathpoint, without any obstacle avoidance.
     *
     * @param robot_id	            The robot id for this agent.
     * @prarm robot_state           The robots current state
     * @param path                  The path of this agent
     * @param radius                The radius of this agent.
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_decel             The maximum deceleration of this agent.
     * @param max_angular_speed     The maximum angular speed of this agent.
     * @param max_angular_accel     The maximum angular acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     */
    LVAgent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path, double radius, double max_speed,
            double max_accel, double max_decel, double max_angular_speed, double max_angular_accel,
            double max_radius_inflation);

    /**
     * Computes the new velocity of this agent.
     *
     * @param agents unused
     * @param the simulators time step
     */
    void computeNewVelocity(const std::map<unsigned int, std::shared_ptr<Agent>> &robots,
                            Duration time_step) override;

    /**
     * Computes the new angular velocity of this agent.
     *
     * @param time_step the time step to use
     */
    void computeNewAngularVelocity(Duration time_step) override;


    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;


    /**
     * Computes the preferred velocity of this agent.
     *
     * @param time_step
     * @return the computed preferred velocity
     */
    Vector computePreferredVelocity(Duration time_step) override;

    /**
     * Update the primitive which this agent is currently pursuing.
     *
     * @param new_primitive The new primitive to pursue
     * @param world The world in which the new primitive is being pursued
     * @param time_step the time_step to use to step at
     */
    void updatePrimitive(const TbotsProto::Primitive &new_primitive, const World &world,
                         Duration time_step) override;
};
