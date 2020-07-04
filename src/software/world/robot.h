#pragma once

#include <optional>

#include "boost/circular_buffer.hpp"
#include "software/new_geom/angle.h"
#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/robot_capabilities.h"
#include "software/world/timestamped_robot_state.h"

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
     * @param history_size The number of previous robot states that should be stored. Must
     * be > 0
     * @param unavailable_capabilities The set of unavailable capabilities for this robot
     */
    explicit Robot(RobotId id, const Point &position, const Vector &velocity,
                   const Angle &orientation, const AngularVelocity &angular_velocity,
                   const Timestamp &timestamp, unsigned int history_size = 20,
                   const std::set<RobotCapability> &unavailable_capabilities =
                       std::set<RobotCapability>());

    /**
     * Creates a new robot with the given initial state
     *
     * @param id The id of the robot to create
     * @param initial_state The initial state of the robot
     * @param history_size The number of previous robot states that should be stored. Must
     * be > 0
     * @param unavailable_capabilities The set of unavailable capabilities for this robot
     */
    explicit Robot(RobotId id, const TimestampedRobotState &initial_state,
                   unsigned int history_size = 20,
                   const std::set<RobotCapability> &unavailable_capabilities =
                       std::set<RobotCapability>());

    /**
     * Updates the robot with new data, updating the current state as well as the
     * predictive model
     *
     * @param new_robot_state A robot state containing new robot data
     */
    void updateState(const TimestampedRobotState &new_robot_state);

    TimestampedRobotState currentState() const;

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
    RobotId id() const;

    /**
     * Returns the current position of the robot
     *
     * @return the current position of the robot
     */
    Point position() const;

    /**
     * Returns the current velocity of the robot
     *
     * @return the current velocity of the robot
     */
    Vector velocity() const;

    /**
     * Returns the current orientation of the robot
     *
     * @return the current orientation of the robot
     */
    Angle orientation() const;

    /**
     * Returns the current angular velocity of the robot
     *
     * @return the current angular velocity of the robot
     */
    AngularVelocity angularVelocity() const;

    /**
     * Gets the buffer which holds all of the previous states
     *
     * @return circular_buffer containing all previous states up to the history_size field
     * cap
     */
    RobotHistory getPreviousStates() const;

    /**
     * Returns the missing capabilities of the robot
     *
     * @return the missing capabilities of the robot
     */
    const std::set<RobotCapability> &getCapabilitiesBlacklist() const;

    /**
     * Returns all capabilities this robot has
     *
     * @return Returns all capabilities this robot has
     */
    std::set<RobotCapability> getCapabilitiesWhitelist() const;

    /**
     * Returns the mutable hardware capabilities of the robot
     *
     * @return the mutable hardware capabilities of the robot
     */
    std::set<RobotCapability> &getMutableRobotCapabilities();

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
    // The id of this robot
    RobotId id_;
    // All previous states of the robot, with the most recent state at the front of the
    // queue, This buffer will never be empty as it's initialized with a RobotState on
    // creation
    // The buffer size (history_size) must be > 0
    RobotHistory states_;
    // The hardware capabilities of the robot, generated from
    // RobotCapabilityFlags::broken_dribblers/chippers/kickers dynamic parameters
    std::set<RobotCapability> unavailable_capabilities_;
};
