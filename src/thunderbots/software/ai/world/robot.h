#pragma once

#include "geom/angle.h"
#include "geom/point.h"

/**
 * Defines an SSL robot
 */
class Robot
{
   public:
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
     * Updates the robot with new data, updating the current state as well as the
     * predictive model
     *
     * @param new_robot_data A robot containing new robot data
     */
    void update(const Robot& new_robot_data);

    /**
     * Returns the id of the robot
     *
     * @return the id of the robot
     */
    unsigned int id() const;

    /**
     * Returns the current position of the robot
     *
     * @return the current position of the robot
     */
    Point position() const;

    /**
     * Returns the estimated position of the robot at a future time, relative to the
     * current time
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's position. For example, a value of 1.5 would return the
     * estimated position of the robot 1.5s in the future.
     *
     * @return the estimated position of the robot at a future time.
     * Coordinates are in metres.
     */
    Point estimatePositionAtFutureTime(double time_delta = 0.0) const;

    /**
     * Returns the current velocity of the robot
     *
     * @return the current velocity of the robot
     */
    Vector velocity() const;

    /**
     * Returns the estimated velocity of the robot at a future time, relative to the
     * current time
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's velocity. For example, a value of 1.5 would return the
     * estimated velocity of the robot 1.5s in the future.
     *
     * @return the estimated velocity of the robot at a future time.
     * Coordinates are in metres.
     */
    Vector estimateVelocityAtFutureTime(double time_delta = 0.0) const;

    /**
     * Returns the current orientation of the robot
     *
     * @return the current orientation of the robot
     */
    Angle orientation() const;

    /**
     * Returns the estimated orientation of the robot at a future time, relative to the
     * current time
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's orientation. For example, a value of 1.5 would return the
     * estimated orientation of the robot 1.5s in the future.
     *
     * @return the estimated orientation of the robot at a future time.
     * Coordinates are in metres.
     */
    Angle estimateOrientationAtFutureTime(double time_delta = 0.0) const;

    /**
     * Returns the current angular velocity of the robot
     *
     * @return the current angular velocity of the robot
     */
    AngularVelocity angularVelocity() const;

    /**
     * Returns the estimated angular velocity of the robot at a future time, relative to
     * the current time
     *
     * @param time_delta The relative amount of time in the future (in seconds) at which
     * to predict the robot's angular velocity. For example, a value of 1.5 would return
     * the estimated angular velocity of the robot 1.5s in the future.
     *
     * @return the estimated angular velocity of the robot at a future time.
     * Coordinates are in metres.
     */
    AngularVelocity estimateAngularVelocityAtFutureTime(double time_delta = 0.0) const;

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
    // The id of this robot
    const unsigned int id_;
    // The current position of the robot, with coordinates in metres
    Point position_;
    // The current velocity of the robot, in metres per second
    Vector velocity_;
    // The current orientation of the robot, in radians
    Angle orientation_;
    // The current angular velocity of the robot, in radians per second
    AngularVelocity angularVelocity_;
};
