#pragma once

#include "proto/vision.pb.h"
#include "shared/constants.h"
#include "shared/robot_constants.h"
#include "shared/robot_constants_2021.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/world/team_types.h"

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
     * Creates a new robot state based on the TbotsProto::RobotState protobuf
     * representation
     *
     * @param robot_state_proto The TbotsProto::RobotState protobuf which this robot state
     * should be based on
     */
    explicit RobotState(const TbotsProto::RobotState &robot_state_proto);

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
};

/**
 * A light structure for a robot state with an ID
 */
struct RobotStateWithId
{
    unsigned int id;
    RobotState robot_state;

    bool operator==(const RobotStateWithId &other) const
    {
        return id == other.id && robot_state == other.robot_state;
    }
};
