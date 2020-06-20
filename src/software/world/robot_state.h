#pragma once

#include <optional>

#include "shared/constants.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/time/duration.h"
#include "software/world/team_colour.h"

using RobotId = unsigned int;

/**
 * This class represents the physical state of a robot
 */
class RobotState
{
   public:
    /**
     * Creates a new robot state with the given position, velocity, orientation, angular
     * velocity, and timestamp
     *
     * @param position The position of the robot, with coordinates in metres
     * @param velocity The velocity of the robot, in metres per second
     * @param orientation The orientation of the robot
     * @param angular_velocity The angular velocity of the robot
     */
    explicit RobotState(const Point &position, const Vector &velocity,
                        const Angle &orientation,
                        const AngularVelocity &angular_velocity);

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
     * Returns if the ball is in the mouth of the robot, i.e. near the dribbler
     *
     * @return ball in mouth
     */
    bool ballInMouth() const;

    /**
     * Set ball in mouth
     *
     * @param ball_in_mouth whether ball is in beam
     */
    void setBallInMouth(bool ball_in_mouth);

    /**
     * Returns time in milliseconds since last chip if available
     *
     * @return ms since last chip
     */
    std::optional<Duration> timeSinceLastChip() const;

    /**
     * Set time in milliseconds since last chip
     *
     * @param time ms since last chip
     */
    void setTimeSinceLastChip(Duration time_ms);

    /**
     * Returns time in milliseconds since last kick if available
     *
     * @return ms since last kick
     */
    std::optional<Duration> timeSinceLastKick() const;

    /**
     * Set time in milliseconds since last kick
     *
     * @param time ms since last kick
     */
    void setTimeSinceLastKick(Duration time_ms);

    /**
     * Defines the equality operator for a RobotState. RobotStates are equal if
     * all their members are equal
     *
     * @param other The robot state to compare against for equality
     *
     * @return True if the other robot state is equal to this robot state, and false
     * otherwise
     */
    bool operator==(const RobotState &other) const;

    /**
     * Defines the inequality operator for a RobotState.
     *
     * @param other The robot state to compare against for inequality
     *
     * @return True if the other robot state is not equal to this robot state, and false
     * otherwise
     */
    bool operator!=(const RobotState &other) const;

   private:
    Point position_;
    Vector velocity_;
    Angle orientation_;
    AngularVelocity angular_velocity_;
    bool ball_in_mouth_;
    std::optional<Duration> time_since_last_chip_ms_;
    std::optional<Duration> time_since_last_kick_ms_;
};

/**
 * A light structure for a robot state with an ID
 */
struct RobotStateWithId
{
    unsigned int id;
    RobotState robot_state;
};
