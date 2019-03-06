#pragma once

#include "geom/angle.h"
#include "geom/point.h"
#include "util/timestamp.h"

/**
 * Defines an SSL robot
 */
class Robot
{
   public:
    /**
     * Creates a new robot given robot data
     *
     * @param id The id of the robot to create
     * @param position the new position of the robot. Coordinates are in metres.
     * @param velocity the new velocity of the robot, in metres / second.
     * @param orientation the new orientation of the robot, in Radians.
     * @param angular_velocity the new angular velocity of the robot, in Radians
     * per second
     * @param timestamp The timestamp at which the robot was observed to be in the given
     * state
     */
    explicit Robot(unsigned int id, const Point &position, const Vector &velocity,
                   const Angle &orientation, const AngularVelocity &angular_velocity,
                   const Timestamp &timestamp);

    /**
     * Updates the state of the robot.
     *
     * @throws std::invalid_argument if the robot is updated with a time from the past
     * @param new_position the new position of the robot. Coordinates are in metres.
     * @param new_velocity the new velocity of the robot, in metres / second.
     * @param new_orientation the new orientation of the robot, in Radians.
     * @param new_angular_velocity the new angular velocity of the robot, in Radians
     * per second
     * @param timestamp The timestamp at which the robot was observed to be in the given
     * state. The timestamp must be >= the robot's latest update timestamp
     */
    void updateState(const Point &new_position, const Vector &new_velocity,
                     const Angle &new_orientation,
                     const AngularVelocity &new_angular_velocity,
                     const Timestamp &timestamp);

    /**
     * Updates the robot with new data, updating the current state as well as the
     * predictive model
     *
     * @param new_robot_data A robot containing new robot data
     */
    void updateState(const Robot &new_robot_data);

    /**
     * Updates the robot's state to be its predicted state at the given timestamp.
     * The timestamp must be >= the robot's last update timestamp
     *
     * @throws std::invalid_argument if the robot is updated with a time from the past
     * @param timestamp The timestamp at which to update the robot's state to. Must
     * be >= the robot's last update timestamp
     */
    void updateStateToPredictedState(const Timestamp &timestamp);

    /**
     * Returns the timestamp for when this robot's data was last updated
     *
     * @return the timestamp for when this robot's data was last updated
     */
    Timestamp lastUpdateTimestamp() const;

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
     * Returns the estimated position of the robot at a future time, relative to when
     * the robot was last updated
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the robot's position. Value must be >= 0.
     * For example, a value of 1.5 seconds would return the estimated position of the
     * robot 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the robot is estimating the position with a time
     * from the past
     * @return the estimated position of the robot at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Point estimatePositionAtFutureTime(const Duration &duration_in_future) const;

    /**
     * Returns the current velocity of the robot
     *
     * @return the current velocity of the robot
     */
    Vector velocity() const;

    /**
     * Returns the estimated velocity of the robot at a future time, relative to when
     * the robot was last updated
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the robot's velocity. Value must be >= 0.
     * For example, a value of 1.5 seconds would return the estimated velocity of the
     * robot 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the robot is estimating the velocity with a time
     * from the past
     * @return the estimated velocity of the robot at the given number of milliseconds
     * in the future, in metres per second
     */
    Vector estimateVelocityAtFutureTime(const Duration &duration_in_future) const;

    /**
     * Returns the current orientation of the robot
     *
     * @return the current orientation of the robot
     */
    Angle orientation() const;

    /**
     * Returns the estimated orientation of the robot at a future time, relative to when
     * the robot was last updated
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the robot's orientation. Value must be >= 0.
     * For example, a value of 1.5 seconds would return the estimated orientation of the
     * robot 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the robot is estimating the orientation with a
     * time from the past
     * @return the estimated orientation of the robot at the given number of milliseconds
     * in the future. Coordinates are in metres.
     */
    Angle estimateOrientationAtFutureTime(const Duration &duration_in_future) const;

    /**
     * Returns the current angular velocity of the robot
     *
     * @return the current angular velocity of the robot
     */
    AngularVelocity angularVelocity() const;

    /**
     * Returns the estimated angular velocity of the robot at a future time, relative to
     * when the robot was last updated
     *
     * @param duration_in_future The relative amount of time in the future
     * at which to predict the robot's angular velocity. Value must be
     * >= 0. For example, a value of 1.5 seconds would return the estimated angular
     * velocity of the robot 1.5 seconds in the future.
     *
     * @throws std::invalid_argument if the robot is estimating the angular velocity with
     * a time from the past
     * @return the estimated angular velocity of the robot at the given number of
     * milliseconds in the future. Coordinates are in metres. Coordinates are in metres.
     */
    AngularVelocity estimateAngularVelocityAtFutureTime(
        const Duration &duration_in_future) const;

    /**
     * Defines the equality operator for a Robot. Robots are equal if their IDs and
     * all other parameters (position, orientation, etc) are equal. The last update
     * timestamp is not part of the equality.
     *
     * @param other The robot to compare against for equality
     * @return True if the other robot is equal to this robot, and false otherwise
     */
    bool operator==(const Robot &other) const;

    /**
     * Defines the inequality operator for a Robot.
     *
     * @param other The robot to compare against for inequality
     * @return True if the other robot is not equal to this robots, and false otherwise
     */
    bool operator!=(const Robot &other) const;

   private:
    // The id of this robot
    unsigned int id_;
    // The current position of the robot, with coordinates in metres
    Point position_;
    // The current velocity of the robot, in metres per second
    Vector velocity_;
    // The current orientation of the robot, in radians
    Angle orientation_;
    // The current angular velocity of the robot, in radians per second
    AngularVelocity angularVelocity_;
    // The timestamp for when this Robot was last updated
    Timestamp last_update_timestamp;
};
