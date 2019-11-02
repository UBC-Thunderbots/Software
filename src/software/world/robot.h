#pragma once

#include <optional>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/util/time/timestamp.h"
#include "software/world/robot_capabilities.h"

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
     * @param history_duration The number of previous robot states that should be stored.
     */
    explicit Robot(unsigned int id, const Point &position, const Vector &velocity,
                   const Angle &orientation, const AngularVelocity &angular_velocity,
                   const Timestamp &timestamp, unsigned int history_duration = 20,
                   const std::set<RobotCapabilities::Capability> &capabilities =
                       RobotCapabilities::allCapabilities());

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
     * @param timestamp The timestamp at which to update the robot's state to. Must
     * be >= the robot's last update timestamp
     * @throws std::invalid_argument if the robot is updated with a time from the past
     */
    void updateStateToPredictedState(const Timestamp &timestamp);

    /**
     * Updates the robot's state to be its predicted state at the given duration from the
     * last time it was updated.
     *
     * @param duration_in_future A duration >= 0.0 that indicates how long in the future
     * (from the last time this robot was updated) to update the robot state by
     * @throws std::invalid_argument if the duration given is negative
     */
    void updateStateToPredictedState(const Duration &duration_in_future);

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
     * Gets the buffer which holds all the previous position states of the robot
     *
     * @return Vector containing the position history starting with the oldest available
     * data at index 0
     */
    std::vector<Point> getPreviousPositions() const;

    /**
     * Gets the buffer which holds all the previous velocity states of the robot
     *
     * @return Vector containing the velocity history starting with the oldest available
     * data at index 0
     */
    std::vector<Vector> getPreviousVelocities() const;

    /**
     * Gets the buffer which holds all the previous orientation states of the robot
     *
     * @return Vector containing the orientation history starting with the oldest
     * available data at index 0
     */
    std::vector<Angle> getPreviousOrientations() const;

    /**
     * Gets the buffer which holds all the previous angular velocity states of the robot
     *
     * @return Vector containing the angular velocity history starting with the oldest
     * available data at index 0
     */
    std::vector<AngularVelocity> getPreviousAngularVelocities() const;

    /**
     * Gets the buffer which holds all the timestamps of the previous states
     *
     * @return Vector containing the update timestamp history starting with the oldest
     * available data at index 0
     */
    std::vector<Timestamp> getPreviousTimestamps() const;

    /**
     * Finds an update timestamp that is close to the provided timestamp and returns the
     * index of the timestamp in the history buffer.
     *
     * @param timestamp timestamp of the update state index we wish to fetch
     * @return Index of the robot's update timestamp closest to the desired time or a
     * std::nullopt if there is not matching timestamp.
     */
    std::optional<int> getHistoryIndexFromTimestamp(Timestamp &timestamp) const;

    /**
     * Returns the most Timestamp corresponding to the most recent update to Robot object
     *
     * @return Timestamp : The Timestamp corresponding to the most recent update to the
     * Robot object
     */
    Timestamp getMostRecentTimestamp() const;

    /**
     * Returns the hardware capabilities of the robot
     *
     * @return the hardware capabilities of the robot
     */
    const std::set<RobotCapabilities::Capability> &getRobotCapabilities() const;

    /**
     * Returns the mutable hardware capabilities of the robot
     *
     * @return the mutable hardware capabilities of the robot
     */
    std::set<RobotCapabilities::Capability> &getMutableRobotCapabilities();

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

    // A comparator for the Robot class that compares Robots by ID. This is equivalent
    // to the "less-than" operator.
    // This comparator is necessary for the Robot class to be used as a key in maps. See
    // https://stackoverflow.com/questions/6573225/what-requirements-must-stdmap-key-classes-meet-to-be-valid-keys
    // and
    // https://stackoverflow.com/questions/5733254/how-can-i-create-my-own-comparator-for-a-map
    //
    // We define this "custom" comparator rather than define the '<' operator for this
    // class because there are many possible ways to order robots, so it doesn't make
    // sense to define a single "normal" way with the '<' operator. Defining this
    // comparator struct lets us use it explicitly when necessary and maintain multiple
    // ways of comparing robots
    struct cmpRobotByID
    {
        bool operator()(const Robot &r1, const Robot &r2) const
        {
            return r1.id() < r2.id();
        }
    };

   private:
    /**
     * Adds a state to the front of the circular buffers storing the state histories of
     * the robot.
     *
     * @param position Position of robot.
     * @param velocity Velocity of robot
     * @param orientation Orientation of robot.
     * @param angular_velocity Angular velocity of robot
     * @param timestamp Time that the robot was in this state.
     */
    void addStateToRobotHistory(const Point &position, const Vector &velocity,
                                const Angle &orientation,
                                const AngularVelocity &angular_velocity,
                                const Timestamp &timestamp);

    // The id of this robot
    unsigned int id_;
    // All previous positions of the robot, with the most recent position at the front of
    // the queue, coordinates in meters
    boost::circular_buffer<Point> positions_;
    // All previous velocities of the robot, with the most recent velocity at the front of
    // the queue, in metres per second
    boost::circular_buffer<Vector> velocities_;
    // All previous orientations of the robot, with the most recent orientation at the
    // front of the queue, in radians
    boost::circular_buffer<Angle> orientations_;
    // All previous angular velocities of the robot, with the most recent angular velocity
    // at the front of the queue, in radians per second
    boost::circular_buffer<AngularVelocity> angularVelocities_;
    // All previous timestamps of when the robot was updated, with the most recent
    // timestamp at the front of the queue,
    boost::circular_buffer<Timestamp> last_update_timestamps;
    // The hardware capabilities of the robot, generated from
    // RobotCapabilityFlags::broken_dribblers/chippers/kickers dynamic parameters
    std::set<RobotCapabilities::Capability> capabilities_;
};
