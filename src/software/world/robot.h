#pragma once

#include <optional>

#include "software/time/timestamp.h"
#include "software/world/robot_capabilities.h"
#include "software/world/robot_state.h"

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
     * @param unavailable_capabilities The set of unavailable capabilities for this robot
     */
    explicit Robot(RobotId id, const Point &position, const Vector &velocity,
                   const Angle &orientation, const AngularVelocity &angular_velocity,
                   const Timestamp &timestamp,
                   const std::set<RobotCapability> &unavailable_capabilities =
                       std::set<RobotCapability>());

    /**
     * Creates a new robot with the given initial state
     *
     * @param id The id of the robot to create
     * @param initial_state The initial state of the robot
     * @param timestamp The timestamp at which the robot was observed to be in the given
     * state
     * @param unavailable_capabilities The set of unavailable capabilities for this robot
     */
    explicit Robot(RobotId id, const RobotState &initial_state,
                   const Timestamp &timestamp,
                   const std::set<RobotCapability> &unavailable_capabilities =
                       std::set<RobotCapability>());

    /**
     * Updates the robot with new data
     *
     * @param robot_state A robot state containing new robot data
     * @param timestamp New timestamp
     */
    void updateState(const RobotState &state, const Timestamp &timestamp);

    /**
     * Gets the current state of the robot
     *
     * @return current state of the robot
     */
    RobotState currentState() const;

    /**
     * Returns the current timestamp of this robot
     *
     * @return the current timestamp
     */
    Timestamp timestamp() const;

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
     * Returns the missing capabilities of the robot
     *
     * @return the missing capabilities of the robot
     */
    const std::set<RobotCapability> &getUnavailableCapabilities() const;

    /**
     * Returns all available capabilities this robot has
     *
     * @return Returns all available capabilities this robot has
     */
    std::set<RobotCapability> getAvailableCapabilities() const;

    /**
     * Returns the mutable hardware capabilities of the robot
     *
     * @return the mutable hardware capabilities of the robot
     */
    std::set<RobotCapability> &getMutableRobotCapabilities();

    /**
     * Decides if a point is near the dribbler of the robot
     *
     * @param test_point The point to check
     *
     * @return whether the test_point is near the dribbler of the robot
     */
    bool isNearDribbler(const Point &test_point) const;

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
    RobotState current_state_;
    Timestamp timestamp_;
    // The hardware capabilities of the robot, generated from
    // RobotCapabilityFlags::broken_dribblers/chippers/kickers dynamic parameters
    std::set<RobotCapability> unavailable_capabilities_;
};
