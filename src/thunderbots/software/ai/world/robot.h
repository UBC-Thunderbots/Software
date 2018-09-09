#pragma once

#include "ai/world/ball.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "thunderbots_msgs/Robot.h"
//
/**
 * Defines an SSL robot
 */
class Robot
{
   public:
    // The maximum radius of an SSL Robot according to the rules
    static const constexpr double MAX_RADIUS = 0.09;

    /**
     * Creates a new robot given a pattern id
     *
     * @param id The id of the robot to create
     */
    explicit Robot(unsigned int id);

    /**
     * Creates a new robot given robot data
     *
     * @param id The id of the robot to create
     * @param position the new position of the robot. Coordinates are in metres.
     * @param velocity the new velocity of the robot, in metres / second.
     * @param orientation the new orientation of the robot, in Radians.
     * @param angular_velocity the new angular velocity of the robot, in Radians
     * per second
    */
    explicit Robot(unsigned int id, const Point& position, const Vector& velocity,
                   const Angle& orientation, const AngularVelocity& angular_velocity);

    /**
     * Creates a new robot given a Robot message
     *
     * @param robot_msg the Robot message containing the new robot information
     */
    explicit Robot(const thunderbots_msgs::Robot& robot_msg);

    /**
     * Updates the state of the robot.
     *
     * @param new_position the new position of the robot. Coordinates are in metres.
     * @param new_velocity the new velocity of the robot, in metres / second.
     * @param new_orientation the new orientation of the robot, in Radians.
     * @param new_angular_velocity the new angular velocity of the robot, in Radians
     * per second
     */
    void update(const Point& new_position, const Vector& new_velocity,
                const Angle& new_orientation,
                const AngularVelocity& new_angular_velocity);

    /**
     * Returns the id of the robot
     *
     * @return the id of the robot
     */
    unsigned int id() const;

    /**
     * Get the predicted position of the robot at a time relative to the current time.
     * Using the default of 0 will give the current position of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's position.
     *
     * @return the predicted position of the robot. Coordinates are in metres.
     */
    Point position(double time_delta = 0.0) const;

    /**
     * Get the predicted velocity of the robot at a time relative to the current time.
     * Using the default of 0 will give the current velocity of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's position.
     *
     * @return the predicted velocity of the robot, in metres / second.
     */
    Vector velocity(double time_delta = 0.0) const;

    /**
     * Get the predicted orientation of the robot at a time relative to the current time.
     * Using the default of 0 will give the current orientation of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's position.
     *
     * @return the predicted orientation of the robot, in Radians.
     */
    Angle orientation(double time_delta = 0.0) const;

    /**
     * Get the predicted angular velocity of the robot at a time relative to the current
     * time. Using the default of 0 will give the current orientation of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's position.
     *
     * @return the predicted angular velocity of the robot, in Radians / second.
     */
    AngularVelocity angularVelocity(double time_delta = 0.0) const;

    /**
     * Defines the equality operator for a Robot. Robots are equal if their IDs and
     * all other parameters (position, orientation, etc) are equal
     *
     * @param other The robot to compare against for equality
     * @return True if the other robot is equal to this robot, and false otherwise
     */
    bool operator==(const Robot& other) const;

    /**
     * Defines the inequality operator for a Robot.
     *
     * @param other The robot to compare against for inequality
     * @return True if the other robot is not equal to this robots, and false otherwise
     */
    bool operator!=(const Robot& other) const;

   private:
    const unsigned int id_;
    Point position_;
    Vector velocity_;
    Angle orientation_;
    AngularVelocity angularVelocity_;
};
