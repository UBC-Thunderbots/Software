#pragma once


#include "software/ai/navigator/path_planner/hrvo/robot_path.h"

class Agent {
public:

    Agent(double min_radius, double max_speed, double max_accel, double max_radius_inflation);

    virtual ~Agent() = default;

    /**
     * Update the agent's radius based on its current velocity
     */
    virtual void updateRadiusFromVelocity() = 0;

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


