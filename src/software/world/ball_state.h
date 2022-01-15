#pragma once

#include "proto/vision.pb.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"

/**
 * Represents the state of a ball
 */
class BallState
{
   public:
    BallState() = delete;

    /**
     * Creates a new ball state with the given position and velocity
     *
     * @param position The position of the ball, with coordinates in metres
     * @param velocity The velocity of the ball, in metres per second
     * @param distance_from_ground The distance of the bottom of the ball from the ground
     * in metres. For example, a ball rolling along the ground has a distance from ground
     * of 0
     */
    explicit BallState(const Point& position, const Vector& velocity,
                       double distance_from_ground = 0.0);

    /**
     * Creates a new ball state based on the TbotsProto::BallState protobuf representation
     *
     * @param ball_state_proto The TbotsProto::BallState protobuf which this ball state
     * should be based on
     */
    explicit BallState(const TbotsProto::BallState& ball_state_proto);

    /**
     * Returns the position of the ball represented by this state
     *
     * @return the position of the ball represented by this state
     */
    Point position() const;

    /**
     * Returns the velocity of the ball represented by this state
     *
     * @return the velocity of the ball represented by this state
     */
    Vector velocity() const;

    /**
     * Returns the distance of the bottom of the ball from the ground in metres
     *
     * @return the distance of the bottom of the ball from the ground in metres
     */
    double distanceFromGround() const;

    /**
     * Defines the equality operator for a BallState. BallStates are equal if their
     * positions and velocities are the same
     *
     * @param other The ball state to compare against for equality
     * @return True if the other ball state is equal to this ball state, and false
     * otherwise
     */
    bool operator==(const BallState& other) const;

    /**
     * Defines the inequality operator for a BallState.
     *
     * @param other The ball state to compare against for inequality
     * @return True if the other ball state is not equal to this ball state, and false
     * otherwise
     */
    bool operator!=(const BallState& other) const;

   private:
    Point position_;
    Vector velocity_;
    double height_;
};
