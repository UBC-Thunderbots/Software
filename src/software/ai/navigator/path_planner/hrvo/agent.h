#pragma once

#include <map>

#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/world.h"
#include "proto/primitive.pb.h"


class Agent {
public:

    Agent(RobotId robot_id, const RobotState &robot_state, TeamSide side, RobotPath &path,
          double radius, double max_speed, double max_accel, double max_radius_inflation);

    virtual ~Agent() = default;

    /**
     * Update the agent's radius based on its current velocity
     */
    void updateRadiusFromVelocity();

    virtual Vector computePreferredVelocity(Duration time_step) = 0;

    virtual void computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots, Duration time_step) = 0;

    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;

    virtual void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                 const World &world,
                                 Duration time_step) = 0;


    // GETTERS

    /**
     * Gets the the path for this agent
     * @return Path for this agent
     */
    RobotPath &getPath();

    /**
     * Gets the max speed for this agent
     * @return max speed for this agent
     */
    const double getMaxSpeed() const;

    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    double getMaxAccel() const;

    /**
     * Return the state of the agent
     *
     * @return The state of the agent
     */
    const RobotState &getRobotState();

    Vector getPreferredVelocity() const;

    void setPreferredVelocity(Vector velocity);

    // robot id of this Agent
    RobotId robot_id;
    // current state
    const RobotState robot_state;
    // whether this Agent is FRIENDLY or ENEMY
    TeamSide side;
    // This agent's current actual radius
    double radius;
    // The path of this Agent
    RobotPath path;

    Vector new_velocity;

    double max_speed;
    const double max_accel;
    // The minimum radius which this agent can be
    const double min_radius;
    // The maximum amount which the radius can increase by
    const double max_radius_inflation;

protected:
    Vector preferred_velocity;

};


