#pragma once

#include <map>

#include "proto/primitive.pb.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/world.h"


class Agent
{
   public:
    /**
     * Constructor
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
    Agent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path,
          double radius, double max_speed, double max_accel, double max_decel,
          double max_angular_speed, double max_angular_accel,
          double max_radius_inflation);


    /**
     * Update the agent's path and current state, based on its new_velocity
     *
     * @param time_step the current time step that the simulator is running at
     */
    void update(Duration time_step);


    /**
     * Update the agent's radius based on its current (robot_state) velocity
     */
    void updateRadiusFromVelocity();


    /**
     * Update the primitive which this agent is currently pursuing.
     *
     * @param new_primitive The new primitive to pursue
     * @param world The world in which the new primitive is being pursued
     * @param time_step the time step to use
     */
    virtual void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                 const World &world, Duration time_step) = 0;


    /**
     * Computes the preferred velocity of this agent.
     *
     * @param time_step the time step to use
     * @return the computed preferred velocity
     */
    virtual Vector computePreferredVelocity(Duration time_step) = 0;


    /**
     * Compute the new velocity that this agent should move at
     *
     * @param robots robots in the simulation
     * @param time_step the time step to use
     */
    virtual void computeNewVelocity(
        const std::map<unsigned int, std::shared_ptr<Agent>> &robots,
        Duration time_step) = 0;

    /**
     * Compute the new angular velocity that this agent should rotate at
     *
     * @param time_step the time step to use
     */
    virtual void computeNewAngularVelocity(Duration time_step) = 0;


    /**
     * Create the VO to represent the given agent, relative to this agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    virtual VelocityObstacle createVelocityObstacle(const Agent &other_agent) = 0;


    /**
     * Return the position of the agent
     *
     * @return The position for this agent
     */
    Point getPosition() const;


    /**
     * Set the position of this agent
     *
     * @param The new position for this agent
     */
    void setPosition(const Point &new_position);


    /**
     * Return the velocity of the agent
     *
     * @return The velocity for this agent
     */
    Vector getVelocity() const;


    /**
     * Set the velocity of the agent
     *
     * @param new_velocity The velocity to set for this agent
     */
    void setVelocity(const Vector &velocity_update);


    /**
     * Return the orientation of the agent
     *
     * @return The orientation for this agent
     */
    Angle getOrientation() const;


    /**
     * Set the orientation of this agent
     *
     * @param The new orientation for this agent
     */
    void setOrientation(const Angle &new_orientation);


    /**
     * Return the angular velocity of the agent
     *
     * @return The angular velocity for this agent
     */
    AngularVelocity getAngularVelocity() const;


    /**
     * Set the angular velocity for this agent
     *
     * @param The new angular velocity for this agent
     */
    void setAngularVelocity(const AngularVelocity &new_angular_velocity);


    /**
     * Get the preferred velocity for this robot
     *
     * @return the computed preferred velocity, if there is one.
     */
    Vector getPreferredVelocity() const;


    /**
     * Set this robots preferred velocity
     *
     * @param pref_velocity The velocity for which preferred_velocity should be set to
     */
    void setPreferredVelocity(const Vector &pref_velocity);


    /**
     * Gets the max speed for this agent
     *
     * @return max speed for this agent
     */
    double getMaxSpeed() const;


    /**
     * Gets the radius for this agent
     *
     * @return radius for this agent
     */
    double getRadius() const;


    /**
     * Gets the the path for this agent
     *
     * @return Path for this agent
     */
    const RobotPath &getPath();


    /**
     * Return the max acceleration of the agent
     *
     * @return The max acceleration of the agent
     */
    double getMaxAccel() const;

   protected:
    // robot id of this agent
    RobotId robot_id;

    // The path of this agent
    RobotPath path;

    // This agent's current radius
    double radius;

    // The minimum radius which this agent can be
    const double min_radius;

    // the max_speed that the agent may move at. This is NOT a physical
    // limitation of the robot, but instead software limitation set through move
    // primitives.
    double max_speed;

    // the maximum acceleration for the agent
    const double max_accel;

    // the maximum deceleration for the agent
    const double max_decel;

    // the maximum speed for the agent
    double max_angular_speed;

    // the maximum acceleration for the agent
    const double max_angular_accel;

    // The maximum amount which the radius can increase by
    const double max_radius_inflation;

    // the computed new_velocity that the robot should pursue
    // this velocity is most optimal for getting to the destination,
    // after considering obstacle avoidance.
    // It may be normalized to max_speed in `update()`.
    Vector new_velocity;

   protected:
    // the preferred velocity the agent should use when finding a new_velocity
    // This velocity represents what's most optimal for getting to the destination
    Vector preferred_velocity;

    Point position;

    Vector velocity;

    Angle orientation;

    AngularVelocity angular_velocity;
};
