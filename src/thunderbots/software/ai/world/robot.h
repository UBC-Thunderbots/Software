#pragma once

#include "geom/angle.h"
#include "geom/point.h"
#include "thunderbots_msgs/Robot.h"

/**
 * Defines an SSL robot
 */
class Robot
{
   public:
    // The max radius of a robot in metres
    static constexpr double MAX_RADIUS = 0.09;

    /**
     * Creates a new robot given a pattern id
     */
    explicit Robot(unsigned int id);

    /**
     * Updates the state of the robot.
     *
     * @param new_position the new position of the robot. Coordinates are in metres.
     * @param new_velocity the new velocity of the robot, in metres / second.
     * @param new_orientation the new orientation of the robot, in Radians.
     * @param new_angular_velocity the new angular velocity of the robot, in Radians
     * per second
     */
    void update(
        const Point& new_position, const Point& new_velocity,
        const Angle& new_orientation, const Angle& new_angular_velocity);

    /**
     * Updates the state of the robot.
     *
     * @param robot_msg the Robot message containing the new data to update with
     */
    void update(const thunderbots_msgs::Robot& robot_msg);

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
     * to predict the ball's position.
     *
     * @return the predicted position of the robot. Coordinates are in metres.
     */
    Point position(double time_delta = 0.0) const;

    /**
     * Get the predicted velocity of the robot at a time relative to the current time.
     * Using the default of 0 will give the current velocity of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the ball's position.
     *
     * @return the predicted velocity of the robot, in metres / second.
     */
    Point velocity(double time_delta = 0.0) const;

    /**
     * Get the predicted orientation of the robot at a time relative to the current time.
     * Using the default of 0 will give the current orientation of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the ball's position.
     *
     * @return the predicted orientation of the robot, in Radians.
     */
    Angle orientation(double time_delta = 0.0) const;

    /**
     * Get the predicted angular velocity of the robot at a time relative to the current
     * time. Using the default of 0 will give the current orientation of the robot.
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the ball's position.
     *
     * @return the predicted angular velocity of the robot, in Radians / second.
     */
    Angle angularVelocity(double time_delta = 0.0) const;

   private:
    const unsigned int id_;
    Point position_;
    Point velocity_;
    Angle orientation_;
    Angle angularVelocity_;
};
