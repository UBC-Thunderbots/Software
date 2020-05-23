#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/timestamp.h"
#include "software/world/robot_state.h"

/**
 * This class represents the state of a robot at a single point in time
 */
class TimestampedRobotState
{
   public:
    /**
     * Creates a new robot state with the given initial state and timestamp
     *
     * @param position The position of the robot, with coordinates in metres
     * @param velocity The velocity of the robot, in metres per second
     * @param orientation The orientation of the robot
     * @param angular_velocity The angular velocity of the robot
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     */
    explicit TimestampedRobotState(const Point &position, const Vector &velocity,
                                   const Angle &orientation,
                                   const AngularVelocity &angular_velocity,
                                   const Timestamp &timestamp);

    /**
     * Creates a new robot state with the given initial state and timestamp
     *
     * @param robot_state The initial state
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     */
    explicit TimestampedRobotState(const RobotState &robot_state,
                                   const Timestamp &timestamp);

    /**
     * Returns the timestamp of the robot represented by this state
     *
     * @return the timestamp of the robot represented by this state
     */
    Timestamp timestamp() const;

    /**
     * Returns the robot state without any timestamp information.
     *
     * @return the robot state without any timestamp information
     */
    RobotState robotState() const;

    /**
     * Defines the equality operator for a TimestampedRobotState.
     * RobotStateWithTimestamps are equal if their positions, velocities,
     * orientation, and angular velocity are the same.
     *
     * @param other The robot state to compare against for equality
     * @return True if the other robot state is equal to this robot state, and false
     * otherwise
     */
    bool operator==(const TimestampedRobotState &other) const;

    /**
     * Defines the inequality operator for a TimestampedRobotState.
     *
     * @param other The robot state to compare against for inequality
     * @return True if the other robot state is not equal to this robot state, and false
     * otherwise
     */
    bool operator!=(const TimestampedRobotState &other) const;

   private:
    Timestamp timestamp_;
    RobotState robot_state_;
};
