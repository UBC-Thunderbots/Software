#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/point.h"
#include "software/util/time/timestamp.h"
#include "software/world/robot_capabilities.h"

class RobotState final
{
   public:
    /**
     * Creates a new robot state with the given position, velocity, orientation, angular
     * velocity, and timestamp
     *
     * @param position The position of the robot, with coordinates in metres
     * @param velocity The velocity of the robot, in metres per second
     * @param orientation The orientation of the robot, in Radians.
     * @param angular_velocity The angular velocity of the robot, in Radians
     * per second
     * @param timestamp The timestamp at which the ball was observed to be at the
     * given position and velocity
     */
    explicit RobotState(const Point &position, const Vector &velocity,
                        const Angle &orientation, const AngularVelocity &angular_velocity,
                        const Timestamp &timestamp);

    /**
     * Returns the position of the robot represented by this state
     *
     * @return the position of the robot represented by this state
     */
    Point position() const;

    /**
     * Returns the velocity of the robot represented by this state
     *
     * @return the velocity of the robot represented by this state
     */
    Vector velocity() const;

    /**
     * Returns the orientation of the robot represented by this state
     *
     * @return the orientation of the robot represented by this state
     */
    Angle orientation() const;

    /**
     * Returns the angular velocity of the robot represented by this state
     *
     * @return the angular velocity of the robot represented by this state
     */
    AngularVelocity angularVelocity() const;

    /**
     * Returns the timestamp of the robot represented by this state
     *
     * @return the timestamp of the robot represented by this state
     */
    Timestamp timestamp() const;

    /**
     * Defines the equality operator for a RobotState. RobotStates are equal if their
     * positions, velocities, orientation, and angular velocity are the same
     *
     * @param other The robot state to compare against for equality
     * @return True if the other robot state is equal to this robot state, and false
     * otherwise
     */
    bool operator==(const RobotState &other) const;

    /**
     * Defines the inequality operator for a RobotState.
     *
     * @param other The robot state to compare against for inequality
     * @return True if the other robot state is not equal to this robot state, and false
     * otherwise
     */
    bool operator!=(const RobotState &other) const;

   private:
    Point position_;

    Vector velocity_;

    Angle orientation_;

    AngularVelocity angular_velocity_;

    Timestamp timestamp_;
};
