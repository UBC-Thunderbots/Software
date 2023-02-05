#pragma once

#include <map>

#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"


class Agent {
public:

    Agent(double min_radius, double max_speed, double max_accel, double max_radius_inflation);

    virtual ~Agent() = default;

    /**
     * Update the agent's radius based on its current velocity
     */
    virtual void updateRadiusFromVelocity() = 0;

    virtual Vector computePreferredVelocity(double time_step) = 0;

    virtual void computeNewVelocity(std::map<unsigned int, Agent> &robots, Duration time_step) = 0;

    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;

    virtual std::vector<VelocityObstacle> computeVelocityObstacles(std::map<int, Agent> &robots, double time_step) = 0;



    // GETTERS N SHIT

    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    double getMaxAccel() const;

    /**
     * Gets the the path for this agent
     * @return Path for this agent
     */
    virtual const RobotPath &getPath() = 0;

    /**
     * Gets the max speed for this agent
     * @return max speed for this agent
     */
    double getMaxSpeed() const;


    // remove depending on simulator impl
    // The minimum radius which this agent can be
    const double min_radius;
    // The maximum amount which the radius can increase by
    const double max_radius_inflation;
    const double max_speed;
    const double max_accel;
};


